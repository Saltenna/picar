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
      // Remove broken client
      streamClients.splice(i, 1);
    }
  }
}

function startFFmpeg() {
  if (ffmpegProcess) return;

  jpegBuffer = Buffer.alloc(0);
  console.log('Starting shared ffmpeg stream...');
  ffmpegProcess = spawn('ffmpeg', [
    '-f', 'v4l2',
    '-framerate', '15',
    '-video_size', '640x480',
    '-i', '/dev/video0',
    '-fflags', 'nobuffer',
    '-flags', 'low_delay',
    '-f', 'mjpeg',
    '-q:v', '5',
    'pipe:1'
  ]);

  ffmpegProcess.stdout.on('data', (chunk) => {
    // Accumulate data and extract complete JPEG frames
    jpegBuffer = Buffer.concat([jpegBuffer, chunk]);

    while (true) {
      const startIdx = jpegBuffer.indexOf(JPEG_START);
      if (startIdx === -1) {
        // No JPEG start found, discard buffer
        jpegBuffer = Buffer.alloc(0);
        break;
      }
      // Discard anything before the JPEG start
      if (startIdx > 0) {
        jpegBuffer = jpegBuffer.subarray(startIdx);
      }

      // Look for JPEG end marker (after the start)
      const endIdx = jpegBuffer.indexOf(JPEG_END, 2);
      if (endIdx === -1) {
        // Incomplete frame, wait for more data
        break;
      }

      // Extract complete frame (including the 2-byte end marker)
      const frame = jpegBuffer.subarray(0, endIdx + 2);
      broadcastFrame(frame);

      // Remove this frame from the buffer
      jpegBuffer = jpegBuffer.subarray(endIdx + 2);
    }
  });

  ffmpegProcess.stderr.on('data', (data) => {
    // ffmpeg logs to stderr; uncomment to debug:
    // console.log('ffmpeg:', data.toString());
  });

  ffmpegProcess.on('close', (code) => {
    console.log(`ffmpeg exited with code ${code}`);
    ffmpegProcess = null;
    jpegBuffer = Buffer.alloc(0);
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
      stopFFmpegIfNoClients();
    });

    startFFmpeg();
  } else {
    // Cert-acceptance landing page: user visits this once to trust the cert
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

