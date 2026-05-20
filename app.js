// app.js
const fs            = require('fs');
const https         = require('https');
const { Server }    = require('socket.io');
const url           = require('url');
const static        = require('node-static');
const { spawn, execSync } = require('child_process');
const { WebSocket, WebSocketServer } = require('ws');
const path          = require('path');

// ── Config ────────────────────────────────────────────────────────────────────
const configPath = path.join(__dirname, 'picar-cfg.json');
let config;
try {
  config = JSON.parse(fs.readFileSync(configPath));
} catch (err) {
  console.error(`Failed to read config file at ${configPath}:`, err);
  process.exit(1);
}

const PWMDriver = require('./pwm_servo');
const pwm = PWMDriver(config);

const file = new static.Server();
const options = {
  key:  fs.readFileSync('./certs/key.pem'),
  cert: fs.readFileSync('./certs/cert.pem'),
};

// ── Stream codec config ───────────────────────────────────────────────────────
// stream_codec: 'webrtc' → WebRTC via MediaMTX (https://host:8889/cam)
// stream_codec: 'h264'   → H264 Annex-B over WebSocket (wss://host:8081/stream)
// stream_codec: 'mjpeg'  → MJPEG multipart over HTTPS (https://host:8081/stream.mjpg)
const streamCodec  = (config.stream_codec || 'h264').toLowerCase();
const H264_WIDTH   = config.h264_width        || 640;
const H264_HEIGHT  = config.h264_height       || 480;
const H264_FPS     = config.h264_framerate    || 30;
const H264_BITRATE = (config.h264_bitrate_kbps || 600) * 1000;
const H264_INTRA   = config.h264_intra_period || 15;   // keyframe every N frames
const WEBRTC_PROTOCOL = config.webrtc_protocol || 'https';
const WEBRTC_PORT     = config.webrtc_port     || 8889;
const WEBRTC_PATH     = config.webrtc_path     || 'cam';

// ── Auto-detect camera binary ─────────────────────────────────────────────────
// rpicam-vid on Pi 5+, libcamera-vid on Pi 4, error if neither found.
// WebRTC mode lets MediaMTX own the Pi camera, so Node does not spawn a camera.
let cameraCmd = null;
if (streamCodec !== 'webrtc') {
  for (const cmd of ['rpicam-vid', 'libcamera-vid']) {
    try { execSync(`which ${cmd}`, { stdio: 'ignore' }); cameraCmd = cmd; break; }
    catch (_) {}
  }
  if (!cameraCmd) console.error('Neither rpicam-vid nor libcamera-vid found on this system');
}
console.log(`Camera command : ${streamCodec === 'webrtc' ? 'MediaMTX rpiCamera source' : (cameraCmd || '(none found)')}`);
console.log(`Stream codec   : ${streamCodec}`);

// ── Shared stream state ───────────────────────────────────────────────────────
let ffmpegProcess = null;
let streamClients = [];          // MJPEG HTTP multipart response objects
const wsClients   = new Set();   // H264 WebSocket connections — have received ≥1 keyframe
const wsPending   = new Set();   // H264 WebSocket connections — waiting for first keyframe

// A newly-connected WebSocket must NOT receive delta (P) frames before it has
// seen its first IDR keyframe — WebCodecs rejects out-of-order inter-frames
// and fires a decode error.  We hold new clients in wsPending and only promote
// them to wsClients on the next IDR, so they never see a delta-before-key.
function totalStreamClients() { return streamClients.length + wsClients.size + wsPending.size; }

// ── H264 NAL-unit parser (Annex-B start-code framing) ─────────────────────────
//
// Scans the raw bytes from libcamera-vid for 3- or 4-byte start codes
// (0x000001 / 0x00000001) and groups NAL units into access units:
//
//   Keyframe packet : SPS(7) + PPS(8) + [SEI/AUD] + IDR-slice(5)
//   Delta packet    : [SEI/AUD] + non-IDR-slice(1)
//
// Why group them?  WebCodecs EncodedVideoChunk for a 'key' frame must contain
// the parameter sets (SPS+PPS) so the decoder can (re)configure itself.
class NalParser {
  constructor(onPacket) {
    this.buf      = Buffer.alloc(0);
    this.pending  = [];   // accumulated SPS/PPS/SEI/AUD before next slice
    this.onPacket = onPacket;
  }

  push(chunk) {
    this.buf = Buffer.concat([this.buf, chunk]);
    while (this._extractOne()) { /* drain */ }
  }

  reset() { this.buf = Buffer.alloc(0); this.pending = []; }

  // Find the next Annex-B start code at or after `from`.
  // Returns { pos, len } or null if none found.
  _findSC(from) {
    const b = this.buf;
    for (let i = from; i < b.length - 2; i++) {
      if (b[i] !== 0 || b[i + 1] !== 0) continue;
      if (b[i + 2] === 1)                               return { pos: i, len: 3 };
      if (i + 3 < b.length && b[i + 2] === 0 && b[i + 3] === 1) return { pos: i, len: 4 };
    }
    return null;
  }

  // Extract one NAL unit from the buffer.  Returns true if one was consumed.
  _extractOne() {
    const sc1 = this._findSC(0);
    if (!sc1) { this.buf = Buffer.alloc(0); return false; }

    const nalStart = sc1.pos + sc1.len;
    const sc2      = this._findSC(nalStart + 1);
    if (!sc2) {
      // Incomplete NAL – keep everything from sc1 onwards for next push()
      if (sc1.pos > 0) this.buf = this.buf.slice(sc1.pos);
      return false;
    }

    const rawNal  = this.buf.slice(sc1.pos, sc2.pos);
    const nalType = this.buf[nalStart] & 0x1f;
    this.buf = this.buf.slice(sc2.pos);

    switch (nalType) {
      case 7:  // SPS
      case 8:  // PPS
      case 6:  // SEI
      case 9:  // AUD (access unit delimiter)
        this.pending.push(rawNal);
        break;
      case 5:  // IDR slice → emit keyframe
        this.pending.push(rawNal);
        this.onPacket(Buffer.concat(this.pending), /*isKeyframe=*/true);
        this.pending = [];
        break;
      case 1:  // Non-IDR slice → emit delta frame
        {
          const all = [...this.pending, rawNal];
          this.pending = [];
          this.onPacket(Buffer.concat(all), /*isKeyframe=*/false);
        }
        break;
      default:
        this.pending.push(rawNal);   // filler / reserved – pass through
    }
    return true;
  }
}

// ── H264 broadcast ────────────────────────────────────────────────────────────
//
// Binary packet layout:
//   Byte 0      : flags  (bit-0 = isKeyframe)
//   Bytes 1–4   : frameCount (uint32 big-endian, wraps at 2^32)
//   Bytes 5–end : H264 Annex-B data (one access unit, start-codes intact)
//
let h264FrameCount = 0;

function broadcastH264(data, isKeyframe) {
  if (!wsClients.size && !wsPending.size) return;
  const hdr = Buffer.allocUnsafe(5);
  hdr[0] = isKeyframe ? 0x01 : 0x00;
  hdr.writeUInt32BE(h264FrameCount, 1);
  h264FrameCount = (h264FrameCount + 1) >>> 0;   // safe uint32 wrap
  const pkt = Buffer.concat([hdr, data]);

  // Send to established clients (have already decoded at least one keyframe)
  for (const ws of wsClients) {
    if (ws.readyState === WebSocket.OPEN) {
      try { ws.send(pkt, { binary: true }); }
      catch (e) { wsClients.delete(ws); }
    }
  }

  // Promote pending clients on the first keyframe they receive
  if (isKeyframe && wsPending.size) {
    for (const ws of wsPending) {
      wsPending.delete(ws);
      if (ws.readyState === WebSocket.OPEN) {
        try { ws.send(pkt, { binary: true }); wsClients.add(ws); }
        catch (_) {}
      }
    }
  }
}

// ── MJPEG broadcast ───────────────────────────────────────────────────────────
const JPEG_START = Buffer.from([0xff, 0xd8]);
const JPEG_END   = Buffer.from([0xff, 0xd9]);
let   jpegBuffer = Buffer.alloc(0);

function broadcastMjpeg(frame) {
  const hdr = `--ffserver\r\nContent-Type: image/jpeg\r\nContent-Length: ${frame.length}\r\n\r\n`;
  for (let i = streamClients.length - 1; i >= 0; i--) {
    const c = streamClients[i];
    try { if (!c.writableEnded) { c.write(hdr); c.write(frame); c.write('\r\n'); } }
    catch (_) { streamClients.splice(i, 1); }
  }
}

// ── Camera lifecycle ──────────────────────────────────────────────────────────
function stopCameraIfNoClients() {
  if (totalStreamClients() === 0 && ffmpegProcess) {
    console.log('No stream clients — stopping camera');
    ffmpegProcess.kill('SIGTERM');
    ffmpegProcess = null;
  }
}

function startCamera() {
  if (streamCodec === 'webrtc') return;
  if (ffmpegProcess) return;
  if (!cameraCmd) { console.error('No camera command available'); return; }
  if (streamCodec === 'h264') _startH264Camera();
  else if (streamCodec === 'mjpeg') _startMjpegCamera();
  else console.error(`Unknown stream_codec "${streamCodec}"`);
}

function _startH264Camera() {
  let gotFirst = false;

  // Key encoder flags for ultra-low latency over bandwidth-limited radio:
  //   --profile baseline  → no B-frames; smallest decode complexity
  //   --intra N           → IDR every N frames (0.5 s at 30 fps)
  //                         limits freeze duration after packet loss to ≤ 0.5 s
  //   --bitrate            → CBR keeps frame sizes predictable; even an IDR
  //                         frame stays ≤ ~8 KB at 600 kbps → < 130 ms on air
  const args = [
    '--codec',     'h264',
    '--width',     String(H264_WIDTH),
    '--height',    String(H264_HEIGHT),
    '--framerate', String(H264_FPS),
    '--bitrate',   String(H264_BITRATE),
    '--intra',     String(H264_INTRA),
    '--profile',   'baseline',
    '--nopreview',
    '-t', '0',
    '-o', '-',
  ];

  console.log(`Starting H264 ${H264_WIDTH}×${H264_HEIGHT}@${H264_FPS}fps ` +
              `${H264_BITRATE / 1000}kbps intra=${H264_INTRA}`);
  ffmpegProcess = spawn(cameraCmd, args);

  const parser = new NalParser((data, isKey) => broadcastH264(data, isKey));

  ffmpegProcess.stdout.on('data', (chunk) => {
    if (!gotFirst) { gotFirst = true; console.log('Camera: first H264 data, stream live'); }
    parser.push(chunk);
  });

  ffmpegProcess.stderr.on('data', (d) => console.log('camera:', d.toString().trim()));

  ffmpegProcess.on('close', (code) => {
    console.log(`Camera exited (code ${code})`);
    ffmpegProcess = null;
    parser.reset();
    if (totalStreamClients() > 0) {
      const delay = gotFirst ? 1000 : 5000;
      console.log(`Restarting camera in ${delay} ms…`);
      setTimeout(startCamera, delay);
    }
  });

  ffmpegProcess.on('error', (e) => {
    console.error('Camera spawn error:', e.message);
    ffmpegProcess = null;
  });
}

function _startMjpegCamera() {
  jpegBuffer = Buffer.alloc(0);
  let gotFirst = false;

  const args = [
    '--codec',     'mjpeg',
    '--width',     '480',
    '--height',    '360',
    '--framerate', '12',
    '--quality',   '20',
    '--nopreview',
    '-t', '0',
    '-o', '-',
  ];

  console.log('Starting MJPEG camera stream…');
  ffmpegProcess = spawn(cameraCmd, args);

  ffmpegProcess.stdout.on('data', (chunk) => {
    if (!gotFirst) { gotFirst = true; console.log('Camera: first MJPEG frame, stream live'); }
    jpegBuffer = Buffer.concat([jpegBuffer, chunk]);
    while (true) {
      const s = jpegBuffer.indexOf(JPEG_START);
      if (s === -1) { jpegBuffer = Buffer.alloc(0); break; }
      if (s > 0) jpegBuffer = jpegBuffer.subarray(s);
      const e = jpegBuffer.indexOf(JPEG_END, 2);
      if (e === -1) break;
      broadcastMjpeg(jpegBuffer.subarray(0, e + 2));
      jpegBuffer = jpegBuffer.subarray(e + 2);
    }
  });

  ffmpegProcess.stderr.on('data', (d) => console.log('camera:', d.toString().trim()));

  ffmpegProcess.on('close', (code) => {
    console.log(`Camera exited (code ${code})`);
    ffmpegProcess = null;
    jpegBuffer = Buffer.alloc(0);
    if (totalStreamClients() > 0) {
      const delay = gotFirst ? 1000 : 5000;
      console.log(`Restarting camera in ${delay} ms…`);
      setTimeout(startCamera, delay);
    }
  });

  ffmpegProcess.on('error', (e) => {
    console.error('Camera spawn error:', e.message);
    ffmpegProcess = null;
  });
}

// Force a new SPS+PPS+IDR sequence by briefly restarting the encoder.
// Called when a WebSocket client reports a decode error.
function forceKeyframe() {
  if (!ffmpegProcess) return;
  console.log('Forcing keyframe — restarting encoder');
  ffmpegProcess.kill('SIGTERM');
  ffmpegProcess = null;
  setTimeout(startCamera, 250);
}

// ── Web UI + control server (port 8443) ───────────────────────────────────────
const appServer = https.createServer(options, (req, res) => {
  const parsed = url.parse(req.url, true);
  if (parsed.pathname === '/status') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ status: 'OK', throttle: old_throttle, steering: old_steering }));
  } else {
    file.serve(req, res);
  }
});

const io = new Server(appServer);
appServer.listen(8443, '0.0.0.0');
console.log('Pi Car web server: https://<ip>:8443/socket.html');

const control_neutral  = config.control_neutral  ?? 0;
const input_timeout_ms = config.input_timeout_ms ?? 500;

let old_throttle      = control_neutral;
let old_steering      = control_neutral;
let smoothed_throttle = control_neutral;
let logcount = 0;
let lastAction = null;

const throttle_ramp_up   = 0;
const throttle_ramp_down = 0;

io.on('connection', (socket) => {
  console.log('Control client connected');

  // Push stream config immediately so the client can set up the right decoder
  if (streamCodec === 'webrtc') {
    socket.emit('streamConfig', {
      codec:    'webrtc',
      protocol: WEBRTC_PROTOCOL,
      port:     WEBRTC_PORT,
      path:     WEBRTC_PATH,
    });
  } else {
    socket.emit('streamConfig', {
      codec:  streamCodec === 'h264' ? 'avc1.42001f' : 'mjpeg',
      width:  H264_WIDTH,
      height: H264_HEIGHT,
      fps:    H264_FPS,
    });
  }

  socket.on('arm',    () => { console.log('ARM');    if (typeof pwm.arm    === 'function') pwm.arm();    });
  socket.on('disarm', () => { console.log('DISARM'); if (typeof pwm.disarm === 'function') pwm.disarm(); });

  socket.on('fromclient', (data) => {
    logcount++;
    const throttleCmd = Number.isFinite(data.throttle)
      ? Math.max(-1, Math.min(1, data.throttle)) : control_neutral;
    const steeringCmd = Number.isFinite(data.steering)
      ? Math.max(-1, Math.min(1, data.steering)) : control_neutral;

    old_throttle = throttleCmd;
    old_steering = steeringCmd;

    if (throttle_ramp_up && throttleCmd > smoothed_throttle)
      smoothed_throttle = Math.min(throttleCmd, smoothed_throttle + throttle_ramp_up);
    else if (throttle_ramp_down && throttleCmd < smoothed_throttle)
      smoothed_throttle = Math.max(throttleCmd, smoothed_throttle - throttle_ramp_down);
    else
      smoothed_throttle = throttleCmd;

    if (logcount === 10) logcount = 0;

    pwm.setServoPWM('throttle', smoothed_throttle);
    pwm.setServoPWM('steering', steeringCmd);
    if (data.shift       !== undefined) pwm.setServoPWM('shift',       data.shift);
    if (data.tlock_front !== undefined) pwm.setServoPWM('tlock_front', data.tlock_front);
    if (data.tlock_rear  !== undefined) pwm.setServoPWM('tlock_rear',  data.tlock_rear);

    clearTimeout(lastAction);
    lastAction = setTimeout(() => {
      pwm.setServoPWM('throttle', control_neutral);
      pwm.setServoPWM('steering', control_neutral);
      console.log(`### EMERGENCY STOP (no input for ${input_timeout_ms} ms)`);
    }, input_timeout_ms);
  });
});

process.on('SIGINT', () => {
  pwm.setServoPWM('throttle', control_neutral);
  pwm.setServoPWM('steering', control_neutral);
  if (ffmpegProcess) ffmpegProcess.kill('SIGTERM');
  console.log('\nShutting down');
  process.exit();
});

// ── Video stream server (port 8081) ───────────────────────────────────────────
const streamServer = https.createServer(options, (req, res) => {
  if (req.url === '/stream.mjpg' && streamCodec === 'mjpeg') {
    // MJPEG multipart stream
    res.writeHead(200, {
      'Content-Type':  'multipart/x-mixed-replace; boundary=ffserver',
      'Cache-Control': 'no-cache',
      'Connection':    'close',
      'Pragma':        'no-cache',
      'Access-Control-Allow-Origin': '*',
    });
    streamClients.push(res);
    console.log(`MJPEG client connected (${streamClients.length} total)`);
    req.on('close', () => {
      streamClients = streamClients.filter(c => c !== res);
      console.log(`MJPEG client disconnected (${streamClients.length} remaining)`);
      stopCameraIfNoClients();
    });
    startCamera();
  } else {
    // Certificate-acceptance landing page (serves any plain HTTP request)
    res.writeHead(200, { 'Content-Type': 'text/html', 'Access-Control-Allow-Origin': '*' });
    res.end(`<!DOCTYPE html><html><body style="background:#111;color:#0f0;font-family:sans-serif;\
display:flex;align-items:center;justify-content:center;height:100vh;margin:0">
      <div style="text-align:center">
        <h2>Stream Server Ready</h2>
        <p>Certificate accepted. You can close this tab.</p>
        <script>
          if (window.opener) window.opener.postMessage('stream-cert-ok', '*');
          setTimeout(() => window.close(), 1500);
        </script>
      </div>
    </body></html>`);
  }
});

// ── H264 WebSocket endpoint  wss://host:8081/stream ───────────────────────────
const wss = new WebSocketServer({ server: streamServer, path: '/stream' });

wss.on('connection', (ws) => {
  // New client goes into wsPending — it will be promoted to wsClients
  // the moment the NAL parser emits the next IDR keyframe.
  wsPending.add(ws);
  console.log(`H264 WebSocket client connected (pending keyframe)`);

  ws.on('message', (msg) => {
    try {
      const d = JSON.parse(msg);
      // requestKeyframe: rate-limited to once per 5 s to prevent restart loops
      if (d.type === 'requestKeyframe') {
        const now = Date.now();
        if (!wss._lastKfForce || now - wss._lastKfForce > 5000) {
          wss._lastKfForce = now;
          forceKeyframe();
        }
      }
    } catch (_) {}
  });

  ws.on('close', () => {
    const wasPending = wsPending.delete(ws);
    wsClients.delete(ws);
    console.log(`H264 WebSocket disconnected (${wsClients.size} active, ${wsPending.size} pending)`);
    stopCameraIfNoClients();
  });

  ws.on('error', (e) => {
    console.error('WS client error:', e.message);
    wsPending.delete(ws);
    wsClients.delete(ws);
  });

  startCamera();
});

streamServer.listen(8081, '0.0.0.0');
const streamUrl = streamCodec === 'h264'
  ? 'wss://<ip>:8081/stream  (H264 WebSocket)'
  : streamCodec === 'webrtc'
    ? `${WEBRTC_PROTOCOL}://<ip>:${WEBRTC_PORT}/${WEBRTC_PATH}/whep  (WebRTC via MediaMTX)`
    : 'https://<ip>:8081/stream.mjpg  (MJPEG)';
console.log(`Video stream: ${streamUrl}`);
