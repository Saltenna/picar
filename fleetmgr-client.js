// fleet-heartbeat.js — periodic heartbeat to Fleet Manager
// Status bitmask: bit 0 = battery trouble, bit 1 = not mobile
'use strict';

const http = require('http');
const os   = require('os');

let _statusBits = 0x00;

function getLocalIp() {
  for (const ifaces of Object.values(os.networkInterfaces())) {
    for (const iface of ifaces) {
      if (iface.family === 'IPv4' && !iface.internal) return iface.address;
    }
  }
  return '127.0.0.1';
}

function setStatusBit(bit, value) {
  if (value) _statusBits |=  (1 << bit);
  else       _statusBits &= ~(1 << bit);
}

function start(config) {
  if (config.fleet_enabled === false) return;
  const fleetUrl = config.fleetManagerUrl;
  if (!fleetUrl) return;

  const roverId = config.rover_id ?? 1;
  const intervalMs = (config.fleet_heartbeat_interval_s || 5) * 1000;

  function send() {
    const payload = JSON.stringify({
      id:        roverId,
      ip:        getLocalIp(),
      timestamp: Math.floor(Date.now() / 1000),
      status:    _statusBits,
    });

    try {
      const u = new URL('/api/heartbeat', fleetUrl);
      const req = http.request({
        hostname: u.hostname,
        port:     u.port || 80,
        path:     u.pathname,
        method:   'POST',
        headers:  {
          'Content-Type':   'application/json',
          'Content-Length': Buffer.byteLength(payload),
        },
      }, res => res.resume());
      req.on('error', e => console.error('Fleet heartbeat error:', e.message));
      req.write(payload);
      req.end();
    } catch (e) {
      console.error('Fleet heartbeat error:', e.message);
    }
  }

  send();
  setInterval(send, intervalMs);
  console.log(`Fleet heartbeat: ${fleetUrl} rover_id=${roverId} every ${intervalMs/1000}s`);
}

module.exports = { start, setStatusBit };
