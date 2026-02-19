// app.js
const fs = require('fs');
const https = require('https');
const { Server } = require('socket.io');
const url = require('url');
const static = require('node-static');
const { spawn } = require('child_process');

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

// --- Shared MJPEG stream (single ffmpeg process) ---
let streamClients = [];
let ffmpegProcess = null;

function startFFmpeg() {
  if (ffmpegProcess) return;

  console.log('Starting shared ffmpeg stream...');
  ffmpegProcess = spawn('ffmpeg', [
    '-f', 'v4l2',
    '-framerate', '15',
    '-video_size', '640x480',
    '-i', '/dev/video0',
    '-fflags', 'nobuffer',
    '-f', 'mjpeg',
    '-q:v', '5',
    'pipe:1'
  ]);

  ffmpegProcess.stdout.on('data', (chunk) => {
    const header = `--ffserver\r\nContent-Type: image/jpeg\r\nContent-Length: ${chunk.length}\r\n\r\n`;
    for (let i = streamClients.length - 1; i >= 0; i--) {
      const client = streamClients[i];
      if (!client.writableEnded) {
        client.write(header);
        client.write(chunk);
        client.write('\r\n');
      }
    }
  });

  ffmpegProcess.stderr.on('data', (data) => {
    // ffmpeg logs to stderr; ignore unless debugging
  });

  ffmpegProcess.on('close', (code) => {
    console.log(`ffmpeg exited with code ${code}`);
    ffmpegProcess = null;
    // If clients are still connected, restart
    if (streamClients.length > 0) {
      setTimeout(startFFmpeg, 1000);
    }
  });

  ffmpegProcess.on('error', (err) => {
    console.error('ffmpeg error:', err.message);
    ffmpegProcess = null;
  });
}

function stopFFmpegIfNoClients() {
  if (streamClients.length === 0 && ffmpegProcess) {
    console.log('No stream clients, stopping ffmpeg');
    ffmpegProcess.kill('SIGTERM');
    ffmpegProcess = null;
  }
}

// Web UI + Socket Server + MJPEG Stream (port 8443)
const appServer = https.createServer(options, (req, res) => {
  const parsedUrl = url.parse(req.url, true);
  if (parsedUrl.pathname === '/status') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ status: 'OK', throttle: old_throttle, steering: old_steering }));
  } else if (parsedUrl.pathname === '/stream.mjpg') {
    res.writeHead(200, {
      'Content-Type': 'multipart/x-mixed-replace; boundary=ffserver',
      'Cache-Control': 'no-cache',
      'Connection': 'close',
      'Pragma': 'no-cache',
    });

    streamClients.push(res);
    console.log(`Stream client connected (${streamClients.length} total)`);

    req.on('close', () => {
      streamClients = streamClients.filter(c => c !== res);
      console.log(`Stream client disconnected (${streamClients.length} remaining)`);
      stopFFmpegIfNoClients();
    });

    // Start ffmpeg if not already running
    startFFmpeg();
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

console.log('MJPEG stream available at https://<ip>:8443/stream.mjpg');

