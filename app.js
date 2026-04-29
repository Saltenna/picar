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

// --- Shared MJPEG stream (single camera process, port 8081) ---
let streamClients = [];
let ffmpegProcess = null;

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

// JPEG markers
const JPEG_START = Buffer.from([0xFF, 0xD8]);
const JPEG_END = Buffer.from([0xFF, 0xD9]);
let jpegBuffer = Buffer.alloc(0);

function broadcastFrame(frameData) {
  const header = `--ffserver\r\nContent-Type: image/jpeg\r\nContent-Length: ${frameData.length}\r\n\r\n`;
  for (let i = streamClients.length - 1; i >= 0; i--) {
    const client = streamClients[i];
    try {
      if (!client.writableEnded) {
        client.write(header);
        client.write(frameData);
        client.write('\r\n');
      }
    } catch (e) {
      streamClients.splice(i, 1);
    }
  }
}

function startCamera() {
  if (ffmpegProcess) return;
  if (!cameraCmd) {
    console.error('No camera command available, cannot start stream');
    return;
  }

  jpegBuffer = Buffer.alloc(0);
  let gotFirstFrame = false;

  const cameraArgs = [
    '--codec', 'mjpeg',
    '--width', '640',
    '--height', '480',
    '--framerate', '15',
    '--quality', '50',
    '--nopreview',
    '-t', '0',
    '-o', '-'
  ];

  console.log(`Starting MJPEG camera stream via ${cameraCmd}...`);
  ffmpegProcess = spawn(cameraCmd, cameraArgs);

  ffmpegProcess.stdout.on('data', (chunk) => {
    if (!gotFirstFrame) {
      gotFirstFrame = true;
      console.log('Camera: first MJPEG frame received, stream is live');
    }
    // Accumulate data and extract complete JPEG frames
    jpegBuffer = Buffer.concat([jpegBuffer, chunk]);

    while (true) {
      const startIdx = jpegBuffer.indexOf(JPEG_START);
      if (startIdx === -1) {
        jpegBuffer = Buffer.alloc(0);
        break;
      }
      if (startIdx > 0) {
        jpegBuffer = jpegBuffer.subarray(startIdx);
      }
      const endIdx = jpegBuffer.indexOf(JPEG_END, 2);
      if (endIdx === -1) break;

      const frame = jpegBuffer.subarray(0, endIdx + 2);
      broadcastFrame(frame);
      jpegBuffer = jpegBuffer.subarray(endIdx + 2);
    }
  });

  ffmpegProcess.stderr.on('data', (data) => {
    console.log('camera:', data.toString().trim());
  });

  ffmpegProcess.on('close', (code) => {
    console.log(`Camera exited with code ${code}`);
    ffmpegProcess = null;
    jpegBuffer = Buffer.alloc(0);
    if (streamClients.length > 0) {
      const delay = gotFirstFrame ? 1000 : 5000;
      console.log(`Restarting camera in ${delay}ms...`);
      setTimeout(startCamera, delay);
    }
  });

  ffmpegProcess.on('error', (err) => {
    console.error('Camera spawn error:', err.message);
    ffmpegProcess = null;
  });
}

function stopCameraIfNoClients() {
  if (streamClients.length === 0 && ffmpegProcess) {
    console.log('No stream clients, stopping camera');
    ffmpegProcess.kill('SIGTERM');
    ffmpegProcess = null;
  }
}

// Web UI + Socket Server (port 8443)
const appServer = https.createServer(options, (req, res) => {
  const parsedUrl = url.parse(req.url, true);
  if (parsedUrl.pathname === '/status') {
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
  if (ffmpegProcess) ffmpegProcess.kill('SIGTERM');
  console.log('\nGracefully shutting down from SIGINT');
  process.exit();
});

console.log('MJPEG stream will start when a client connects to port 8081');

// --- MJPEG Stream Server (port 8081, separate from controls) ---
const streamServer = https.createServer(options, (req, res) => {
  if (req.url === '/stream.mjpg') {
    res.writeHead(200, {
      'Content-Type': 'multipart/x-mixed-replace; boundary=ffserver',
      'Cache-Control': 'no-cache',
      'Connection': 'close',
      'Pragma': 'no-cache',
      'Access-Control-Allow-Origin': '*',
    });

    streamClients.push(res);
    console.log(`Stream client connected (${streamClients.length} total)`);

    req.on('close', () => {
      streamClients = streamClients.filter(c => c !== res);
      console.log(`Stream client disconnected (${streamClients.length} remaining)`);
      stopCameraIfNoClients();
    });

    startCamera();
  } else {
    // Cert-acceptance landing page
    res.writeHead(200, {
      'Content-Type': 'text/html',
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
  }
});

streamServer.listen(8081, '0.0.0.0');
console.log('MJPEG stream available at https://<ip>:8081/stream.mjpg');

