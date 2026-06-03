// streams/mjpeg.js — MJPEG multipart stream over HTTPS (https://host:8081/stream.mjpg)
'use strict';

const { spawn, execSync } = require('child_process');

const JPEG_START = Buffer.from([0xff, 0xd8]);
const JPEG_END   = Buffer.from([0xff, 0xd9]);

module.exports = function createMjpegStream(config, streamServer) {
  const WIDTH   = config.mjpeg_width     || 480;
  const HEIGHT  = config.mjpeg_height    || 360;
  const FPS     = config.mjpeg_framerate || 12;
  const QUALITY = config.mjpeg_quality   || 20;

  let cameraCmd = null;
  for (const cmd of ['rpicam-vid', 'libcamera-vid']) {
    try { execSync(`which ${cmd}`, { stdio: 'ignore' }); cameraCmd = cmd; break; }
    catch (_) {}
  }
  if (!cameraCmd) console.error('MJPEG: neither rpicam-vid nor libcamera-vid found');
  console.log(`MJPEG camera command: ${cameraCmd || '(none found)'}`);

  let clients    = [];   // HTTP response objects
  let cameraProc = null;
  let jpegBuf    = Buffer.alloc(0);

  function clientCount() { return clients.length; }

  function broadcast(frame) {
    const hdr = `--ffserver\r\nContent-Type: image/jpeg\r\nContent-Length: ${frame.length}\r\n\r\n`;
    for (let i = clients.length - 1; i >= 0; i--) {
      const c = clients[i];
      try { if (!c.writableEnded) { c.write(hdr); c.write(frame); c.write('\r\n'); } }
      catch (_) { clients.splice(i, 1); }
    }
  }

  function stop() {
    if (cameraProc) { cameraProc.kill('SIGTERM'); cameraProc = null; }
    jpegBuf = Buffer.alloc(0);
  }

  function start() {
    if (cameraProc) return;
    if (!cameraCmd) { console.error('MJPEG: no camera command available'); return; }

    let gotFirst = false;
    jpegBuf = Buffer.alloc(0);

    const args = [
      '--codec',     'mjpeg',
      '--width',     String(WIDTH),
      '--height',    String(HEIGHT),
      '--framerate', String(FPS),
      '--quality',   String(QUALITY),
      '--nopreview',
      '-t', '0',
      '-o', '-',
    ];

    console.log(`MJPEG starting ${WIDTH}×${HEIGHT}@${FPS}fps quality=${QUALITY}`);
    cameraProc = spawn(cameraCmd, args);

    cameraProc.stdout.on('data', (chunk) => {
      if (!gotFirst) { gotFirst = true; console.log('MJPEG: stream live'); }
      jpegBuf = Buffer.concat([jpegBuf, chunk]);
      while (true) {
        const s = jpegBuf.indexOf(JPEG_START);
        if (s === -1) { jpegBuf = Buffer.alloc(0); break; }
        if (s > 0) jpegBuf = jpegBuf.subarray(s);
        const e = jpegBuf.indexOf(JPEG_END, 2);
        if (e === -1) break;
        broadcast(jpegBuf.subarray(0, e + 2));
        jpegBuf = jpegBuf.subarray(e + 2);
      }
    });

    cameraProc.stderr.on('data', (d) => console.log('MJPEG camera:', d.toString().trim()));

    cameraProc.on('close', (code) => {
      console.log(`MJPEG camera exited (code ${code})`);
      cameraProc = null;
      jpegBuf = Buffer.alloc(0);
      if (clientCount() > 0) {
        const delay = gotFirst ? 1000 : 5000;
        console.log(`MJPEG: restarting in ${delay} ms…`);
        setTimeout(start, delay);
      }
    });

    cameraProc.on('error', (e) => {
      console.error('MJPEG camera spawn error:', e.message);
      cameraProc = null;
    });
  }

  // ── HTTP multipart endpoint ───────────────────────────────────────────────
  streamServer.on('request', (req, res) => {
    if (req.url !== '/stream.mjpg') return;
    res.writeHead(200, {
      'Content-Type':  'multipart/x-mixed-replace; boundary=ffserver',
      'Cache-Control': 'no-cache',
      'Connection':    'close',
      'Pragma':        'no-cache',
      'Access-Control-Allow-Origin': '*',
    });
    clients.push(res);
    console.log(`MJPEG client connected (${clients.length} total)`);
    req.on('close', () => {
      clients = clients.filter(c => c !== res);
      console.log(`MJPEG client disconnected (${clients.length} remaining)`);
      if (clientCount() === 0) stop();
    });
    start();
  });

  return {
    clientCount,
    stop,
    getStreamConfig() {
      return { codec: 'mjpeg' };
    },
  };
};
