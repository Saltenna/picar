// streams/h264.js — H264 Annex-B over WebSocket (wss://host:8081/stream)
'use strict';

const { spawn, execSync } = require('child_process');
const { WebSocket, WebSocketServer } = require('ws');

// ── NAL-unit parser (Annex-B start-code framing) ─────────────────────────────
//
// Groups raw bytes from rpicam-vid/libcamera-vid into complete access units:
//   Keyframe packet : SPS(7) + PPS(8) + [SEI/AUD] + IDR-slice(5)
//   Delta packet    : [SEI/AUD] + non-IDR-slice(1)
//
// WebCodecs EncodedVideoChunk for a 'key' frame must include the SPS+PPS so
// the decoder can (re)configure itself — hence the grouping.
class NalParser {
  constructor(onPacket) {
    this.buf      = Buffer.alloc(0);
    this.pending  = [];
    this.onPacket = onPacket;
  }

  push(chunk) {
    this.buf = Buffer.concat([this.buf, chunk]);
    while (this._extractOne()) { /* drain */ }
  }

  reset() { this.buf = Buffer.alloc(0); this.pending = []; }

  _findSC(from) {
    const b = this.buf;
    for (let i = from; i < b.length - 2; i++) {
      if (b[i] !== 0 || b[i + 1] !== 0) continue;
      if (b[i + 2] === 1)                               return { pos: i, len: 3 };
      if (i + 3 < b.length && b[i + 2] === 0 && b[i + 3] === 1) return { pos: i, len: 4 };
    }
    return null;
  }

  _extractOne() {
    const sc1 = this._findSC(0);
    if (!sc1) { this.buf = Buffer.alloc(0); return false; }

    const nalStart = sc1.pos + sc1.len;
    const sc2      = this._findSC(nalStart + 1);
    if (!sc2) {
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
      case 9:  // AUD
        this.pending.push(rawNal);
        break;
      case 5:  // IDR slice → keyframe
        this.pending.push(rawNal);
        this.onPacket(Buffer.concat(this.pending), true);
        this.pending = [];
        break;
      case 1:  // non-IDR slice → delta frame
        {
          const all = [...this.pending, rawNal];
          this.pending = [];
          this.onPacket(Buffer.concat(all), false);
        }
        break;
      default:
        this.pending.push(rawNal);
    }
    return true;
  }
}

// ── Module factory ────────────────────────────────────────────────────────────
module.exports = function createH264Stream(config, streamServer) {
  const WIDTH   = config.h264_width        || 640;
  const HEIGHT  = config.h264_height       || 480;
  const FPS     = config.h264_framerate    || 30;
  const BITRATE = (config.h264_bitrate_kbps || 600) * 1000;
  const INTRA   = config.h264_intra_period || 15;

  // Detect rpicam-vid (Pi 5+) or libcamera-vid (Pi 4)
  let cameraCmd = null;
  for (const cmd of ['rpicam-vid', 'libcamera-vid']) {
    try { execSync(`which ${cmd}`, { stdio: 'ignore' }); cameraCmd = cmd; break; }
    catch (_) {}
  }
  if (!cameraCmd) console.error('H264: neither rpicam-vid nor libcamera-vid found');
  console.log(`H264 camera command: ${cameraCmd || '(none found)'}`);

  // ── WebSocket state ───────────────────────────────────────────────────────
  // New connections go to wsPending until they receive their first IDR keyframe.
  // This prevents WebCodecs from seeing a delta frame before any key frame.
  const wsClients = new Set();
  const wsPending = new Set();
  let frameCount  = 0;
  let cameraProc  = null;

  function clientCount() { return wsClients.size + wsPending.size; }

  function broadcast(data, isKeyframe) {
    if (!wsClients.size && !wsPending.size) return;
    const hdr = Buffer.allocUnsafe(5);
    hdr[0] = isKeyframe ? 0x01 : 0x00;
    hdr.writeUInt32BE(frameCount, 1);
    frameCount = (frameCount + 1) >>> 0;
    const pkt = Buffer.concat([hdr, data]);

    for (const ws of wsClients) {
      if (ws.readyState === WebSocket.OPEN) {
        try { ws.send(pkt, { binary: true }); }
        catch (_) { wsClients.delete(ws); }
      }
    }

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

  // ── Camera lifecycle ──────────────────────────────────────────────────────
  function stop() {
    if (cameraProc) { cameraProc.kill('SIGTERM'); cameraProc = null; }
  }

  function start() {
    if (cameraProc) return;
    if (!cameraCmd) { console.error('H264: no camera command available'); return; }

    let gotFirst = false;
    // --profile baseline: no B-frames; --intra N: IDR every N frames (~0.5 s at
    // 30 fps) so freeze after packet loss is bounded; --bitrate CBR keeps IDR
    // size predictable (≤ ~8 KB at 600 kbps → < 130 ms on air).
    const args = [
      '--codec',     'h264',
      '--width',     String(WIDTH),
      '--height',    String(HEIGHT),
      '--framerate', String(FPS),
      '--bitrate',   String(BITRATE),
      '--intra',     String(INTRA),
      '--profile',   'baseline',
      '--nopreview',
      '-t', '0',
      '-o', '-',
    ];

    console.log(`H264 starting ${WIDTH}×${HEIGHT}@${FPS}fps ${BITRATE/1000}kbps intra=${INTRA}`);
    cameraProc = spawn(cameraCmd, args);

    const parser = new NalParser((data, isKey) => broadcast(data, isKey));

    cameraProc.stdout.on('data', (chunk) => {
      if (!gotFirst) { gotFirst = true; console.log('H264: stream live'); }
      parser.push(chunk);
    });

    cameraProc.stderr.on('data', (d) => console.log('H264 camera:', d.toString().trim()));

    cameraProc.on('close', (code) => {
      console.log(`H264 camera exited (code ${code})`);
      cameraProc = null;
      parser.reset();
      if (clientCount() > 0) {
        const delay = gotFirst ? 1000 : 5000;
        console.log(`H264: restarting in ${delay} ms…`);
        setTimeout(start, delay);
      }
    });

    cameraProc.on('error', (e) => {
      console.error('H264 camera spawn error:', e.message);
      cameraProc = null;
    });
  }

  // Force a new SPS+PPS+IDR by restarting the encoder.
  // Rate-limited by the caller to once per 5 s.
  function forceKeyframe() {
    if (!cameraProc) return;
    console.log('H264: forcing keyframe — restarting encoder');
    stop();
    setTimeout(start, 250);
  }

  // ── WebSocket endpoint ────────────────────────────────────────────────────
  const wss = new WebSocketServer({ server: streamServer, path: '/stream' });

  wss.on('connection', (ws) => {
    wsPending.add(ws);
    console.log(`H264 WS client connected (pending keyframe)`);

    ws.on('message', (msg) => {
      try {
        const d = JSON.parse(msg);
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
      wsPending.delete(ws);
      wsClients.delete(ws);
      console.log(`H264 WS disconnected (${wsClients.size} active, ${wsPending.size} pending)`);
      if (clientCount() === 0) stop();
    });

    ws.on('error', (e) => {
      console.error('H264 WS error:', e.message);
      wsPending.delete(ws);
      wsClients.delete(ws);
    });

    start();
  });

  return {
    clientCount,
    stop,
    getStreamConfig() {
      return { codec: 'avc1.42001f', width: WIDTH, height: HEIGHT, fps: FPS };
    },
  };
};
