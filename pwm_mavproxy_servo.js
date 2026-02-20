// pwm_mavproxy_servo.js
// Sends RC_CHANNELS_OVERRIDE to MAVProxy over TCP
// Uses proper MAVLink v1 framing with CRC

const net = require('net');

const MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70;
const RC_OVERRIDE_CRC_EXTRA = 124;
const HEARTBEAT_CRC_EXTRA = 50;

class PWMMavproxy {
  constructor(config) {
    this.min_us = config.pwm_min_us || 1000;
    this.max_us = config.pwm_max_us || 2000;
    this.neutral = Math.round((this.min_us + this.max_us) / 2);

    this.host = config.mavproxy_host || '127.0.0.1';
    this.port = config.mavproxy_port || 5760;
    this.target_system = config.mavproxy_target_system || 1;
    this.target_component = config.mavproxy_target_component || 1;

    this.seq = 0;
    this.connected = false;
    this.socket = null;

    this.channels = new Uint16Array(8);
    this.channelMap = {
      throttle: 2, // RC channel 3 (0-indexed)
      steering: 0  // RC channel 1 (0-indexed)
    };

    this.rate_hz = config.mavproxy_rate_hz || 20;
    this.interval = null;
    this.heartbeatInterval = null;

    console.log(`MAVProxy PWM driver: TCP to ${this.host}:${this.port}, ` +
      `target sys=${this.target_system} comp=${this.target_component}, ${this.rate_hz}Hz`);

    this.connect();
  }

  connect() {
    if (this.socket) {
      this.socket.removeAllListeners();
      this.socket.destroy();
    }

    this.connected = false;
    console.log(`Connecting to MAVProxy at ${this.host}:${this.port}...`);

    this.socket = net.createConnection({ host: this.host, port: this.port }, () => {
      console.log('Connected to MAVProxy via TCP');
      this.connected = true;
      this.startHeartbeat();
      this.startLoop();
    });

    this.socket.on('data', (data) => {
      // Parse for heartbeat responses (msg id 0)
      for (let i = 0; i < data.length - 5; i++) {
        if (data[i] === 0xFE && data[i + 5] === 0) {
          console.log('MAVProxy: Received vehicle heartbeat');
          break;
        }
      }
    });

    this.socket.on('error', (err) => {
      console.error('MAVProxy TCP error:', err.message);
      this.connected = false;
    });

    this.socket.on('close', () => {
      console.log('MAVProxy TCP connection closed, reconnecting in 3s...');
      this.connected = false;
      if (this.interval) { clearInterval(this.interval); this.interval = null; }
      if (this.heartbeatInterval) { clearInterval(this.heartbeatInterval); this.heartbeatInterval = null; }
      setTimeout(() => this.connect(), 3000);
    });
  }

  sendPacket(buf) {
    if (this.connected && this.socket && !this.socket.destroyed) {
      try {
        this.socket.write(buf);
      } catch (e) {
        console.error('TCP write error:', e.message);
      }
    }
  }

  scale(value) {
    const midpoint = (this.max_us + this.min_us) / 2;
    const range = (this.max_us - this.min_us) / 2;
    return Math.round(midpoint + range * value);
  }

  setServoPWM(name, value) {
    const ch = this.channelMap[name];
    if (ch === undefined) return;
    this.channels[ch] = this.clamp(this.scale(value), this.min_us, this.max_us);
  }

  clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  startHeartbeat() {
    if (this.heartbeatInterval) clearInterval(this.heartbeatInterval);
    const sendHB = () => this.sendPacket(this.buildHeartbeat());
    sendHB();
    this.heartbeatInterval = setInterval(sendHB, 1000);
  }

  buildHeartbeat() {
    const payloadLen = 9;
    const buf = Buffer.alloc(6 + payloadLen + 2);

    let i = 0;
    buf[i++] = 0xFE;
    buf[i++] = payloadLen;
    buf[i++] = this.seq & 0xFF; this.seq++;
    buf[i++] = 255;          // sysid (GCS)
    buf[i++] = 0;            // compid
    buf[i++] = 0;            // msg id (HEARTBEAT)

    buf.writeUInt32LE(0, i); i += 4; // custom_mode
    buf[i++] = 6;            // MAV_TYPE_GCS
    buf[i++] = 8;            // MAV_AUTOPILOT_INVALID
    buf[i++] = 0;            // base_mode
    buf[i++] = 4;            // MAV_STATE_ACTIVE
    buf[i++] = 3;            // mavlink_version

    let crc = PWMMavproxy.crc16(buf.subarray(1, 6 + payloadLen), 5 + payloadLen);
    crc = PWMMavproxy.crcAccumulate(HEARTBEAT_CRC_EXTRA, crc);
    buf.writeUInt16LE(crc, 6 + payloadLen);
    return buf;
  }

  startLoop() {
    if (this.interval) clearInterval(this.interval);
    const period = 1000 / this.rate_hz;
    let logCount = 0;
    this.interval = setInterval(() => {
      this.sendPacket(this.buildRCOverride());
      logCount++;
      if (logCount % (this.rate_hz * 5) === 1) {
        console.log(`RC Override: ch1=${this.channels[0]} ch3=${this.channels[2]} (TCP connected=${this.connected})`);
      }
    }, period);
  }

  static crc16(buf, len) {
    let crc = 0xFFFF;
    for (let i = 0; i < len; i++) {
      let tmp = buf[i] ^ (crc & 0xFF);
      tmp ^= (tmp << 4) & 0xFF;
      crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
      crc &= 0xFFFF;
    }
    return crc;
  }

  static crcAccumulate(byte, crc) {
    let tmp = byte ^ (crc & 0xFF);
    tmp ^= (tmp << 4) & 0xFF;
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    return crc & 0xFFFF;
  }

  buildRCOverride() {
    const payloadLen = 18;
    const buf = Buffer.alloc(6 + payloadLen + 2);

    let i = 0;
    buf[i++] = 0xFE;
    buf[i++] = payloadLen;
    buf[i++] = this.seq & 0xFF; this.seq++;
    buf[i++] = 255;
    buf[i++] = 0;
    buf[i++] = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;

    for (let c = 0; c < 8; c++) {
      buf.writeUInt16LE(this.channels[c] || 0, i);
      i += 2;
    }
    buf[i++] = this.target_system;
    buf[i++] = this.target_component;

    let crc = PWMMavproxy.crc16(buf.subarray(1, 6 + payloadLen), 5 + payloadLen);
    crc = PWMMavproxy.crcAccumulate(RC_OVERRIDE_CRC_EXTRA, crc);
    buf.writeUInt16LE(crc, 6 + payloadLen);
    return buf;
  }
}

module.exports = PWMMavproxy;

