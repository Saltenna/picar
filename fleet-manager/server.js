// fleet-manager/server.js
'use strict';

const http   = require('http');
const fs     = require('fs');
const path   = require('path');
const url    = require('url');

const cfg = JSON.parse(fs.readFileSync(path.join(__dirname, 'fleet-mgr-cfg.json'), 'utf8'));
const PORT              = process.env.FLEET_PORT || cfg.port || 3000;
const OFFLINE_THRESHOLD = (cfg.offline_threshold_s || 15) * 1000;

// In-memory rover registry: { [id]: { id, ip, timestamp, status, lastSeen } }
const rovers = {};

function decodeStatus(bitmask) {
  return {
    batteryTrouble: !!(bitmask & 0x01),
    notMobile:      !!(bitmask & 0x02),
  };
}

function getRoverList() {
  const now = Date.now();
  return Object.values(rovers).map(r => ({
    id:             r.id,
    ip:             r.ip,
    online:         (now - r.lastSeen) < OFFLINE_THRESHOLD,
    lastSeen:       r.lastSeen,
    ...decodeStatus(r.status || 0),
    controllerUrl:  `https://${r.ip}:8443/socket.html`,
  }));
}

function readBody(req) {
  return new Promise((resolve, reject) => {
    let data = '';
    req.on('data', chunk => { data += chunk; });
    req.on('end', () => {
      try { resolve(JSON.parse(data)); }
      catch (e) { reject(e); }
    });
    req.on('error', reject);
  });
}

const server = http.createServer(async (req, res) => {
  const parsed = url.parse(req.url, true);

  // POST /api/heartbeat
  if (req.method === 'POST' && parsed.pathname === '/api/heartbeat') {
    try {
      const body = await readBody(req);
      if (!body.id || !body.ip) {
        res.writeHead(400, { 'Content-Type': 'application/json' });
        return res.end(JSON.stringify({ error: 'missing id or ip' }));
      }
      rovers[body.id] = { ...body, lastSeen: Date.now() };
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ ok: true }));
    } catch (e) {
      res.writeHead(400, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: 'invalid json' }));
    }
    return;
  }

  // GET /api/rovers
  if (req.method === 'GET' && parsed.pathname === '/api/rovers') {
    res.writeHead(200, { 'Content-Type': 'application/json', 'Access-Control-Allow-Origin': '*' });
    return res.end(JSON.stringify(getRoverList()));
  }

  // GET / — serve dashboard
  if (req.method === 'GET' && (parsed.pathname === '/' || parsed.pathname === '/index.html')) {
    const html = fs.readFileSync(path.join(__dirname, 'dashboard.html'), 'utf8');
    res.writeHead(200, { 'Content-Type': 'text/html' });
    return res.end(html);
  }

  res.writeHead(404);
  res.end('Not found');
});

server.listen(PORT, '0.0.0.0', () => {
  console.log(`Fleet Manager running on http://0.0.0.0:${PORT}`);
});
