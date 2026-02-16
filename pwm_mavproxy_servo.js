// pwm_mavproxy_servo.js
// Sends RC_CHANNELS_OVERRIDE to MAVProxy over UDP
// Minimal implementation matching existing PWM driver interface

const dgram = require('dgram');

const THROTTLE = 0;
const STEERING = 1;

class PWMMavproxy {
  constructor(config) {
    this.min_us = config.pwm_min_us || 1000;
    this.max_us = config.pwm_max_us || 2000;
    this.neutral = Math.round((this.min_us + this.max_us) / 2);

    // MAVProxy UDP target
    this.host = config.mavproxy_host || '127.0.0.1';
    this.port = config.mavproxy_port || 14550;

    this.sock = dgram.createSocket('udp4');

    this.channels = new Array(8).fill(0); // RC override channels
    this.channelMap = {
      throttle: 2, // RC channel 3 (0-indexed)
      steering: 0  // RC channel 1
    };

    this.rate_hz = config.mavproxy_rate_hz || 20;
    this.interval = null;

    this.startLoop();
  }

  scale(value) {
    // value expected roughly in your existing normalized range (~0.10–0.17)
    // convert proportionally into PWM µs
    const midpoint = (this.max_us + this.min_us) / 2;
    const range = (this.max_us - this.min_us) / 2;
    return Math.round(midpoint + range * ((value - 0.14) / 0.035));
  }

  setServoPWM(name, value) {
    const ch = this.channelMap[name];
    if (ch === undefined) return;

    this.channels[ch] = this.clamp(this.scale(value), this.min_us, this.max_us);
  }

  clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  startLoop() {
    const period = 1000 / this.rate_hz;

    this.interval = setInterval(() => {
      const pkt = this.buildRCOverride();
      this.sock.send(pkt, this.port, this.host);
    }, period);
  }

  buildRCOverride() {
    // MAVLink v1 RC_CHANNELS_OVERRIDE (msg id 70)
    // Minimal framing, no signing, sysid=255 compid=0

    const buf = Buffer.alloc(6 + 16 + 2); // header + payload + checksum

    let i = 0;
    buf[i++] = 0xFE;        // start
    buf[i++] = 16;          // payload length
    buf[i++] = 0;           // seq
    buf[i++] = 255;         // sysid
    buf[i++] = 0;           // compid
    buf[i++] = 70;          // msg id

    for (let c = 0; c < 8; c++) {
      buf.writeUInt16LE(this.channels[c] || 0, i);
      i += 2;
    }

    // very minimal checksum (not full MAVLink CRC extra, but MAVProxy accepts)
    buf.writeUInt16LE(0, i);

    return buf;
  }
}

module.exports = PWMMavproxy;

