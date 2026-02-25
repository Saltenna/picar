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

// --- Shared H.264 stream (single camera process, streamed via socket.io) ---
let ffmpegProcess = null;
let socketClients = new Set();
let latestKeyframe = null;  // cache last keyframe so new/recovering clients start clean

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

// H.264 NAL unit type detection
function isKeyframe(buf) {
  for (let i = 0; i < buf.length - 4; i++) {
    if (buf[i] === 0 && buf[i+1] === 0) {
      let nalStart;
      if (buf[i+2] === 0 && buf[i+3] === 1) nalStart = i + 4;
      else if (buf[i+2] === 1) nalStart = i + 3;
      else continue;
      if (nalStart < buf.length) {
        const nalType = buf[nalStart] & 0x1f;
        if (nalType === 5 || nalType === 7) return true;
      }
    }
  }
  return false;
}

function startCamera() {
  if (ffmpegProcess) return;
  if (!cameraCmd) {
    console.error('No camera command available, cannot start stream');
    return;
  }

  let gotFirstFrame = false;

  const cameraArgs = [
    '--codec', 'h264',
    '--width', '640',
    '--height', '480',
    '--framerate', '20',
    '--bitrate', '600000',
    '--profile', 'baseline',
    '--intra', '10',
    '--inline',
    '--nopreview',
    '-t', '0',
    '-o', '-'
  ];

  // rpicam-vid (Pi 5) uses libav backend and needs explicit format for stdout
  if (cameraCmd === 'rpicam-vid') {
    cameraArgs.push('--libav-format', 'h264');
  }

  console.log(`Starting H.264 camera stream via ${cameraCmd}...`);
  ffmpegProcess = spawn(cameraCmd, cameraArgs);

  ffmpegProcess.stdout.on('data', (chunk) => {
    if (!gotFirstFrame) {
      gotFirstFrame = true;
      console.log('Camera: first H.264 data received, stream is live');
    }
    const keyframe = isKeyframe(chunk);
    if (keyframe) {
      latestKeyframe = chunk;
    }
    // volatile.emit: always send latest frame, drop if socket can't keep up.
    // Keeps the feed current — an RC car needs "now" not "smooth".
    for (const s of socketClients) {
      s.volatile.emit('h264data', chunk);
    }
  });

  ffmpegProcess.stderr.on('data', (data) => {
    console.log('camera:', data.toString().trim());
  });

  ffmpegProcess.on('close', (code) => {
    console.log(`Camera exited with code ${code}`);
    ffmpegProcess = null;
    if (socketClients.size > 0) {
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
  if (socketClients.size === 0 && ffmpegProcess) {
    console.log('No clients connected, stopping camera');
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

let old_throttle = 0.14;
let old_steering = 0.14;
const pwm_neutral = config.pwm_neutral;
let smoothed_throttle = pwm_neutral;
let logcount = 0;
let lastAction = null;

const throttle_ramp_up = 0.000;
const throttle_ramp_down = 0.000;

io.on('connection', (socket) => {
  console.log('Socket connected');
  socketClients.add(socket);
  startCamera();
  // Send cached keyframe so client can decode immediately
  if (latestKeyframe) {
    socket.emit('h264data', latestKeyframe);
  }

  socket.on('disconnect', () => {
    console.log('Socket disconnected');
    socketClients.delete(socket);
    stopCameraIfNoClients();
  });

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
    old_throttle = data.throttle;
    old_steering = data.steering;

    if (throttle_ramp_up && (data.throttle > smoothed_throttle)) {
      smoothed_throttle += throttle_ramp_up;
      if (smoothed_throttle > data.throttle) smoothed_throttle = data.throttle;
    } else if (throttle_ramp_down && (data.throttle < smoothed_throttle)) {
      smoothed_throttle -= throttle_ramp_down;
      if (smoothed_throttle < data.throttle) smoothed_throttle = data.throttle;
    }
    else {
      smoothed_throttle = data.throttle;
    }

    if (logcount === 10) logcount = 0;

    pwm.setServoPWM('throttle', smoothed_throttle); // PWM0 for throttle
    pwm.setServoPWM('steering', data.steering);     // PWM1 for steering
    if (data.shift !== undefined) {
      pwm.setServoPWM('shift', data.shift);          // RC2 for 2-speed transmission
    }
    if (data.tlock_front !== undefined) {
      pwm.setServoPWM('tlock_front', data.tlock_front); // RC4 for front t-lock diff
    }
    if (data.tlock_rear !== undefined) {
      pwm.setServoPWM('tlock_rear', data.tlock_rear);   // RC5 for rear t-lock diff
    }

    clearInterval(lastAction);
    lastAction = setInterval(() => {
      pwm.setServoPWM('throttle', pwm_neutral);
      pwm.setServoPWM('steering', pwm_neutral);
      console.log('### EMERGENCY STOP');
    }, 2000);
  });
});

process.on('SIGINT', function () {
  pwm.setServoPWM('throttle', pwm_neutral);
  pwm.setServoPWM('steering', pwm_neutral);
  if (ffmpegProcess) ffmpegProcess.kill('SIGTERM');
  console.log('\nGracefully shutting down from SIGINT');
  process.exit();
});

console.log('H.264 stream will start when a client connects via socket.io');

