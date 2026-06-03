// app.js
const fs            = require('fs');
const https         = require('https');
const { Server }    = require('socket.io');
const url           = require('url');
const static        = require('node-static');
const path          = require('path');
const os            = require('os');

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

// ── Video stream server (port 8081) ───────────────────────────────────────────
// Stream modules attach their request/WebSocket handlers first (inside createStream),
// then a fallback listener handles everything else as the cert-acceptance page.
const streamServer = https.createServer(options);

const createStream = require('./streams');
const stream = createStream(config, streamServer);
console.log(`Stream codec: ${(config.stream_codec || 'h264').toLowerCase()}`);

// Fallback: cert-acceptance landing page for any request not handled by the stream module
streamServer.on('request', (req, res) => {
  if (res.headersSent || res.writableEnded) return;
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
});

streamServer.listen(8081, '0.0.0.0');
console.log(`Video stream server: https://<ip>:8081`);

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

// ── Bandwidth / FPS stats ─────────────────────────────────────────────────────
// Reads /proc/net/dev for interface-level RX and TX byte counters.
// TX from stream modules is more accurate for H264/MJPEG (excludes control
// traffic); for WebRTC the module returns null and we fall back to interface TX.
function readNetDev(iface) {
  try {
    for (const line of fs.readFileSync('/proc/net/dev', 'utf8').split('\n')) {
      const t = line.trim();
      if (!t.startsWith(iface + ':')) continue;
      const p = t.split(/\s+/);
      // fields: iface: rxBytes rxPkts rxErrs rxDrop rxFifo rxFrame rxComp rxMcast txBytes ...
      return { rx: parseInt(p[1]), tx: parseInt(p[9]) };
    }
  } catch (_) {}
  return null;
}

function detectStatsIface() {
  // Prefer wlan0/eth0 over loopback; fall back to first non-lo interface.
  const preferred = ['wlan0', 'eth0', 'wlan1'];
  const nets = Object.keys(os.networkInterfaces()).filter(n => n !== 'lo');
  return preferred.find(n => nets.includes(n)) || nets[0] || 'eth0';
}

const statsIface   = config.stats_interface || detectStatsIface();
let   lastNetDev   = readNetDev(statsIface);
console.log(`Stats interface: ${statsIface}`);

setInterval(() => {
  const streamStats = stream.getStats();      // {txBytes, frames} — nulls for WebRTC
  const net         = readNetDev(statsIface);

  let rxKbps = null, txKbps = null, fps = null;

  if (net && lastNetDev) {
    rxKbps = Math.round((net.rx - lastNetDev.rx) * 8 / 1000);
    // Use interface TX for WebRTC (MediaMTX owns the bytes), module TX otherwise
    if (streamStats.txBytes === null) {
      txKbps = Math.round((net.tx - lastNetDev.tx) * 8 / 1000);
    }
  }
  lastNetDev = net;

  if (streamStats.txBytes !== null) txKbps = Math.round(streamStats.txBytes * 8 / 1000);
  if (streamStats.frames  !== null) fps    = streamStats.frames;

  io.emit('stats', { txKbps, rxKbps, fps, iface: statsIface });
}, 1000);

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

  // Push stream config so the client sets up the right decoder
  socket.emit('streamConfig', stream.getStreamConfig());

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
  stream.stop();
  console.log('\nShutting down');
  process.exit();
});
