// pwm_mavproxy_servo.js
// Sends RC_CHANNELS_OVERRIDE to MAVProxy over UDP
// Uses proper MAVLink v1 framing with CRC

const dgram = require('dgram');

const MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70;
const RC_OVERRIDE_CRC_EXTRA = 124; // MAVLink CRC_EXTRA for msg 70

class PWMMavproxy {
  constructor(config) {
    this.min_us = config.pwm_min_us || 1000;
    this.max_us = config.pwm_max_us || 2000;
    this.neutral = Math.round((this.min_us + this.max_us) / 2);

    // MAVProxy UDP target
    this.host = config.mavproxy_host || '127.0.0.1';
    this.port = config.mavproxy_port || 14550;
    this.target_system = config.mavproxy_target_system || 1;
    this.target_component = config.mavproxy_target_component || 1;

    this.sock = dgram.createSocket('udp4');
    this.seq = 0;

    this.channels = new Uint16Array(8); // RC override channels (µs)
    // Initialize all to 0 (0 = "no override" in MAVLink)
    this.channelMap = {
      throttle: 2, // RC channel 3 (0-indexed)
      steering: 0  // RC channel 1 (0-indexed)
    };

    this.rate_hz = config.mavproxy_rate_hz || 20;
    this.interval = null;
    this.heartbeatInterval = null;

    console.log(`MAVProxy PWM driver: sending to ${this.host}:${this.port}, ` +
      `target sys=${this.target_system} comp=${this.target_component}, ${this.rate_hz}Hz`);

    // Listen for responses from MAVProxy (for debugging)
    this.sock.on('message', (msg, rinfo) => {
      if (msg.length >= 6 && msg[0] === 0xFE && msg[5] === 0) {
        // Got a heartbeat back from vehicle
        console.log('MAVProxy: Received heartbeat from vehicle');
      }
    });

    this.startHeartbeat();
    this.startLoop();
  }

  scale(value) {
    // Same as pigpio driver: value is roughly -1..1, map to PWM µs
    // scale(0) = 1500 (neutral), scale(1) = 2000, scale(-1) = 1000
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
    // Send GCS heartbeat every 1 second so MAVProxy registers us as a valid link
    const sendHB = () => {
      const pkt = this.buildHeartbeat();
      this.sock.send(pkt, this.port, this.host, (err) => {
        if (err) console.error('Heartbeat send error:', err.message);
      });
    };
    sendHB(); // Send immediately
    this.heartbeatInterval = setInterval(sendHB, 1000);
  }

  buildHeartbeat() {
    // MAVLink v1 HEARTBEAT (msg id 0)
    // Payload: custom_mode(4) + type(1) + autopilot(1) + base_mode(1) + system_status(1) + mavlink_version(1) = 9 bytes
    const HEARTBEAT_CRC_EXTRA = 50;
    const payloadLen = 9;
    const buf = Buffer.alloc(6 + payloadLen + 2);

    let i = 0;
    buf[i++] = 0xFE;        // MAVLink v1 start
    buf[i++] = payloadLen;   // payload length
    buf[i++] = this.seq & 0xFF;
    this.seq++;
    buf[i++] = 255;          // system id (GCS)
    buf[i++] = 0;            // component id
    buf[i++] = 0;            // msg id (HEARTBEAT)

    // Payload (wire order: largest fields first)
    buf.writeUInt32LE(0, i); i += 4;  // custom_mode
    buf[i++] = 6;            // type = MAV_TYPE_GCS
    buf[i++] = 8;            // autopilot = MAV_AUTOPILOT_INVALID
    buf[i++] = 0;            // base_mode
    buf[i++] = 4;            // system_status = MAV_STATE_ACTIVE
    buf[i++] = 3;            // mavlink_version

    let crc = PWMMavproxy.crc16(buf.subarray(1, 6 + payloadLen), 5 + payloadLen);
    crc = PWMMavproxy.crcAccumulate(HEARTBEAT_CRC_EXTRA, crc);
    buf.writeUInt16LE(crc, 6 + payloadLen);

    return buf;
  }

  startLoop() {
    const period = 1000 / this.rate_hz;
    let logCount = 0;
    this.interval = setInterval(() => {
      const pkt = this.buildRCOverride();
      this.sock.send(pkt, this.port, this.host, (err) => {
        if (err) console.error('UDP send error:', err.message);
      });
      logCount++;
      if (logCount % (this.rate_hz * 5) === 1) { // Log every 5 seconds
        console.log(`RC Override: ch1=${this.channels[0]} ch3=${this.channels[2]} → ${this.host}:${this.port}`);
      }
    }, period);
  }

  // X.25 CRC used by MAVLink v1
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
    // MAVLink v1 RC_CHANNELS_OVERRIDE (msg id 70)
    // Payload: target_system(1) + target_component(1) + 8 x uint16 channels = 18 bytes
    const payloadLen = 18;
    const buf = Buffer.alloc(6 + payloadLen + 2); // header(6) + payload(18) + checksum(2)

    let i = 0;
    buf[i++] = 0xFE;                                // MAVLink v1 start
    buf[i++] = payloadLen;                           // payload length
    buf[i++] = this.seq & 0xFF;                      // sequence number
    this.seq++;
    buf[i++] = 255;                                  // system id (GCS)
    buf[i++] = 0;                                    // component id (GCS)
    buf[i++] = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;  // msg id

    // Payload: channels first (little-endian uint16), then target_system, target_component
    // MAVLink wire order for RC_CHANNELS_OVERRIDE is:
    //   chan1_raw(2) chan2_raw(2) chan3_raw(2) chan4_raw(2)
    //   chan5_raw(2) chan6_raw(2) chan7_raw(2) chan8_raw(2)
    //   target_system(1) target_component(1)
    for (let c = 0; c < 8; c++) {
      buf.writeUInt16LE(this.channels[c] || 0, i);
      i += 2;
    }
    buf[i++] = this.target_system;
    buf[i++] = this.target_component;

    // CRC: computed over bytes 1..(6+payloadLen-1), then accumulate CRC_EXTRA
    let crc = PWMMavproxy.crc16(buf.subarray(1, 6 + payloadLen), 5 + payloadLen);
    crc = PWMMavproxy.crcAccumulate(RC_OVERRIDE_CRC_EXTRA, crc);

    buf.writeUInt16LE(crc, 6 + payloadLen);

    return buf;
  }
}

module.exports = PWMMavproxy;

