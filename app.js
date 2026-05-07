// app.js
const fs = require('fs');
const https = require('https');
const { Server } = require('socket.io');
const url = require('url');
const static = require('node-static');
const { spawn, execSync } = require('child_process');

const path = require('path');
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
  key: fs.readFileSync('./certs/key.pem'),
  cert: fs.readFileSync('./certs/cert.pem'),
};

const hlsDir = '/tmp/picar-hls';
const hlsPlaylist = path.join(hlsDir, 'stream.m3u8');
const hlsSegmentPattern = path.join(hlsDir, 'stream-%03d.ts');
const hlsPlayerPath = require.resolve('hls.js/dist/hls.min.js');
const VIDEO_FRAMERATE = 15;
const STREAM_IDLE_TIMEOUT_MS = 10000;
const STREAM_STOP_TIMEOUT_MS = 5000;
const HLS_READ_RETRY_MS = 100;
const HLS_READ_TIMEOUT_MS = 2500;
fs.mkdirSync(hlsDir, { recursive: true });

// --- Shared H.264/HLS stream (single camera process, port 8081) ---
let cameraProcess = null;
let ffmpegProcess = null;
let streamIdleTimer = null;

// Auto-detect camera command: rpicam-vid (Pi 5+) or libcamera-vid (Pi 4)
let cameraCmd = null;
for (const cmd of ['rpicam-vid', 'libcamera-vid']) {
  try {
    execSync(`which ${cmd}`, { stdio: 'ignore' });
    cameraCmd = cmd;
    break;
  } catch (_) {}
}
if (!cameraCmd) {
  console.error('Neither rpicam-vid nor libcamera-vid found on this system');
}
console.log(`Camera command: ${cameraCmd || '(none found)'}`);

function clearHlsOutput() {
  for (const entry of fs.readdirSync(hlsDir)) {
    if (entry === 'stream.m3u8' || /^stream-\d+\.ts$/.test(entry)) {
      try {
        fs.unlinkSync(path.join(hlsDir, entry));
      } catch (err) {
        if (err.code !== 'ENOENT') {
          console.warn(`Failed to remove stale HLS file ${entry}:`, err.message);
        }
      }
    }
  }
}

function logProcessOutput(prefix, data) {
  const message = data.toString().trim();
  if (message) console.log(`${prefix}: ${message}`);
}

function stopCamera(reason) {
  if (streamIdleTimer) {
    clearTimeout(streamIdleTimer);
    streamIdleTimer = null;
  }

  if (!cameraProcess && !ffmpegProcess) return;
  if (reason) console.log(reason);

  const camera = cameraProcess;
  const ffmpeg = ffmpegProcess;

  if (camera && ffmpeg) {
    try {
      camera.stdout.unpipe(ffmpeg.stdin);
    } catch (_) {}
  }
  if (ffmpeg && ffmpeg.stdin && !ffmpeg.stdin.destroyed) {
    ffmpeg.stdin.end();
  }

  for (const child of [ffmpeg, camera]) {
    if (!child || child.exitCode !== null || child.signalCode !== null) continue;
    let exited = false;
    child.once('exit', () => {
      exited = true;
    });
    child.kill('SIGTERM');
    const forceKillTimer = setTimeout(() => {
      if (!exited && child.exitCode === null && child.signalCode === null) {
        child.kill('SIGKILL');
      }
    }, STREAM_STOP_TIMEOUT_MS);
    forceKillTimer.unref();
  }
}

function startCamera() {
  if (cameraProcess || ffmpegProcess) return true;
  if (!cameraCmd) {
    console.error('No camera command available, cannot start stream');
    return false;
  }

  clearHlsOutput();

  const cameraArgs = [
    '--codec', 'h264',
    '--width', '640',
    '--height', '480',
    '--framerate', String(VIDEO_FRAMERATE),
    '--bitrate', '1500000',
    '--profile', 'baseline',
    '--intra', String(VIDEO_FRAMERATE),
    '--inline',
    '--nopreview',
    '-t', '0',
    '-o', '-'
  ];

  const ffmpegArgs = [
    '-hide_banner',
    '-loglevel', 'warning',
    '-fflags', '+genpts+nobuffer',
    '-flags', 'low_delay',
    '-use_wallclock_as_timestamps', '1',
    '-analyzeduration', '100000',
    '-probesize', '32768',
    '-r', String(VIDEO_FRAMERATE),
    '-f', 'h264',
    '-i', 'pipe:0',
    '-c:v', 'copy',
    '-muxdelay', '0',
    '-muxpreload', '0',
    '-f', 'hls',
    '-hls_time', '1',
    '-hls_list_size', '4',
    '-hls_delete_threshold', '2',
    '-hls_flags', 'delete_segments+omit_endlist+independent_segments',
    '-hls_segment_filename', hlsSegmentPattern,
    hlsPlaylist
  ];

  console.log(`Starting H.264 camera stream via ${cameraCmd}...`);

  const camera = spawn(cameraCmd, cameraArgs, { stdio: ['ignore', 'pipe', 'pipe'] });
  const ffmpeg = spawn('ffmpeg', ffmpegArgs, { stdio: ['pipe', 'ignore', 'pipe'] });

  cameraProcess = camera;
  ffmpegProcess = ffmpeg;

  camera.stdout.pipe(ffmpeg.stdin);
  ffmpeg.stdin.on('error', () => {});

  camera.stderr.on('data', (data) => logProcessOutput('camera', data));
  ffmpeg.stderr.on('data', (data) => logProcessOutput('ffmpeg', data));

  camera.on('error', (err) => {
    console.error('Camera spawn error:', err.message);
    if (cameraProcess === camera) cameraProcess = null;
    stopCamera('Stopping stream after camera error');
  });

  ffmpeg.on('error', (err) => {
    console.error('ffmpeg spawn error:', err.message);
    if (ffmpegProcess === ffmpeg) ffmpegProcess = null;
    stopCamera('Stopping stream after ffmpeg error');
  });

  camera.on('close', (code, signal) => {
    console.log(`Camera exited with code ${code}, signal ${signal || 'none'}`);
    if (cameraProcess === camera) cameraProcess = null;
    if (ffmpegProcess === ffmpeg) stopCamera('Stopping ffmpeg because camera exited');
  });

  ffmpeg.on('close', (code, signal) => {
    console.log(`ffmpeg exited with code ${code}, signal ${signal || 'none'}`);
    if (ffmpegProcess === ffmpeg) ffmpegProcess = null;
    if (cameraProcess === camera) stopCamera('Stopping camera because ffmpeg exited');
  });

  return true;
}

function markStreamActive() {
  const started = startCamera();
  if (!started) return false;

  if (streamIdleTimer) clearTimeout(streamIdleTimer);
  streamIdleTimer = setTimeout(() => {
    streamIdleTimer = null;
    stopCamera(`No HLS requests for ${STREAM_IDLE_TIMEOUT_MS}ms, stopping camera`);
  }, STREAM_IDLE_TIMEOUT_MS);
  streamIdleTimer.unref();

  return true;
}

function readHlsFile(filePath, callback) {
  const deadline = Date.now() + HLS_READ_TIMEOUT_MS;

  function tryRead() {
    fs.readFile(filePath, (err, data) => {
      if (!err) {
        callback(null, data);
        return;
      }
      if (err.code !== 'ENOENT' || Date.now() >= deadline) {
        callback(err);
        return;
      }
      setTimeout(tryRead, HLS_READ_RETRY_MS);
    });
  }

  tryRead();
}

function getHlsContentType(filename) {
  if (filename.endsWith('.m3u8')) return 'application/vnd.apple.mpegurl';
  if (filename.endsWith('.ts')) return 'video/mp2t';
  return 'application/octet-stream';
}

// Web UI + Socket Server (port 8443)
const appServer = https.createServer(options, (req, res) => {
  const parsedUrl = url.parse(req.url, true);
  if (parsedUrl.pathname === '/vendor/hls.min.js') {
    fs.readFile(hlsPlayerPath, (err, data) => {
      if (err) {
        res.writeHead(404);
        res.end();
        return;
      }
      res.writeHead(200, {
        'Content-Type': 'application/javascript; charset=utf-8',
        'Cache-Control': 'no-cache',
      });
      res.end(data);
    });
  } else if (parsedUrl.pathname === '/status') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ status: 'OK', throttle: old_throttle, steering: old_steering }));
  } else {
    file.serve(req, res);
  }
});

const io = new Server(appServer);
appServer.listen(8443, '0.0.0.0');
console.log('Pi Car web server listening on https://<ip>:8443/socket.html');

// App control values are normalized to [-1..1] across all frontends.
const control_neutral = config.control_neutral ?? 0;
const input_timeout_ms = config.input_timeout_ms ?? 500;

let old_throttle = control_neutral;
let old_steering = control_neutral;
let smoothed_throttle = control_neutral;
let logcount = 0;
let lastAction = null;

const throttle_ramp_up = 0.000;
const throttle_ramp_down = 0.000;

// --- Socket.io: C2 controls only (video is on separate port 8081) ---
io.on('connection', (socket) => {
  console.log('Control client connected');

  socket.on('arm', () => {
    console.log('Client requested ARM');
    if (typeof pwm.arm === 'function') pwm.arm();
  });

  socket.on('disarm', () => {
    console.log('Client requested DISARM');
    if (typeof pwm.disarm === 'function') pwm.disarm();
  });

  socket.on('fromclient', (data) => {
    logcount++;
    const throttleCmd = Number.isFinite(data.throttle)
      ? Math.max(-1, Math.min(1, data.throttle))
      : control_neutral;
    const steeringCmd = Number.isFinite(data.steering)
      ? Math.max(-1, Math.min(1, data.steering))
      : control_neutral;

    old_throttle = throttleCmd;
    old_steering = steeringCmd;

    if (throttle_ramp_up && (throttleCmd > smoothed_throttle)) {
      smoothed_throttle += throttle_ramp_up;
      if (smoothed_throttle > throttleCmd) smoothed_throttle = throttleCmd;
    } else if (throttle_ramp_down && (throttleCmd < smoothed_throttle)) {
      smoothed_throttle -= throttle_ramp_down;
      if (smoothed_throttle < throttleCmd) smoothed_throttle = throttleCmd;
    }
    else {
      smoothed_throttle = throttleCmd;
    }

    if (logcount === 10) logcount = 0;

    pwm.setServoPWM('throttle', smoothed_throttle); // PWM0 for throttle
    pwm.setServoPWM('steering', steeringCmd);        // PWM1 for steering
    if (data.shift !== undefined) {
      pwm.setServoPWM('shift', data.shift);          // RC2 for 2-speed transmission
    }
    if (data.tlock_front !== undefined) {
      pwm.setServoPWM('tlock_front', data.tlock_front); // RC4 for front t-lock diff
    }
    if (data.tlock_rear !== undefined) {
      pwm.setServoPWM('tlock_rear', data.tlock_rear);   // RC5 for rear t-lock diff
    }

    clearTimeout(lastAction);
    lastAction = setTimeout(() => {
      pwm.setServoPWM('throttle', control_neutral);
      pwm.setServoPWM('steering', control_neutral);
      console.log(`### EMERGENCY STOP (no input for ${input_timeout_ms}ms)`);
    }, input_timeout_ms);
  });
});

process.on('SIGINT', function () {
  pwm.setServoPWM('throttle', control_neutral);
  pwm.setServoPWM('steering', control_neutral);
  stopCamera();
  console.log('\nGracefully shutting down from SIGINT');
  process.exit();
});

console.log('H.264 HLS stream will start when a client connects to port 8081');

// --- H.264/HLS Stream Server (port 8081, separate from controls) ---
const streamServer = https.createServer(options, (req, res) => {
  const parsedUrl = url.parse(req.url);
  const pathname = parsedUrl.pathname || '/';

  if (req.method === 'OPTIONS') {
    res.writeHead(204, {
      'Access-Control-Allow-Origin': '*',
      'Access-Control-Allow-Methods': 'GET, OPTIONS',
    });
    res.end();
    return;
  }

  if (pathname.startsWith('/hls/')) {
    const filename = path.basename(pathname);
    if (!/^(stream\.m3u8|stream-\d+\.ts)$/.test(filename)) {
      res.writeHead(404, { 'Access-Control-Allow-Origin': '*' });
      res.end();
      return;
    }

    if (!markStreamActive()) {
      res.writeHead(503, {
        'Content-Type': 'text/plain',
        'Cache-Control': 'no-store',
        'Access-Control-Allow-Origin': '*',
      });
      res.end('Camera stream is unavailable');
      return;
    }

    const filePath = path.join(hlsDir, filename);
    readHlsFile(filePath, (err, data) => {
      if (err) {
        res.writeHead(404, {
          'Cache-Control': 'no-store',
          'Access-Control-Allow-Origin': '*',
        });
        res.end();
        return;
      }

      res.writeHead(200, {
        'Content-Type': getHlsContentType(filename),
        'Cache-Control': 'no-store, no-cache, must-revalidate, proxy-revalidate',
        'Pragma': 'no-cache',
        'Expires': '0',
        'Access-Control-Allow-Origin': '*',
      });
      res.end(data);
    });
    return;
  }

  // Cert-acceptance landing page for browsers that require accepting port 8081.
  res.writeHead(200, {
    'Content-Type': 'text/html',
    'Cache-Control': 'no-store',
    'Access-Control-Allow-Origin': '*',
  });
  res.end(`<!DOCTYPE html><html><body style="background:#111;color:#0f0;font-family:sans-serif;display:flex;align-items:center;justify-content:center;height:100vh;margin:0">
    <div style="text-align:center">
      <h2>Stream Server Ready</h2>
      <p>Certificate accepted. You can close this tab.</p>
      <script>
        if (window.opener) { window.opener.postMessage('stream-cert-ok', '*'); }
        setTimeout(() => window.close(), 1500);
      </script>
    </div>
  </body></html>`);
});

streamServer.listen(8081, '0.0.0.0');
console.log('H.264 HLS stream available at https://<ip>:8081/hls/stream.m3u8');
