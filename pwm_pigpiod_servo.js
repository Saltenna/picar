// pwm_pigpiod_servo.js
// This pwm backend drives Raspberry Pi 3 and 4 pigpiod via the gpiod daemon
// you must run: sudo systemctl start pigpiod.service
//

const pigpio = require('pigpio-client').pigpio;
const client = pigpio();

const THROTTLE = 0;
const STEERING = 1;

class PWMPigpioServo {
  constructor(config) {
    this.period_us = config.pwm_period_us;
    this.min_us = config.pwm_min_us;
    this.max_us = config.pwm_max_us;

    this.pins = {
      THROTTLE: config.throttle_gpio,
      STEERING: config.steering_gpio
    };

    this.channelMap = {
      throttle: THROTTLE,
      steering: STEERING
    };

    this.outputs = {};

    console.log(`pins:${this.pins}`);
    for (const [id, pin] of Object.entries(this.pins)) {
      console.log(`loop id:${id} pin:${pin} type:${typeof pin}`);
      if (typeof pin === 'undefined') {
        console.error(`Pin not defined for channel ${id}`);
        continue;
      }

      try {
        console.log('ms1');
        const gpio = client.gpio(pin);
        console.log('ms2');
        gpio.modeSet('output');
        console.log('ms3');
        this.outputs[Number(id)] = gpio;
        console.log(`Configured GPIO${pin} for servo ${id}`);
      } catch (e) {
        console.error(`Failed to configure GPIO${pin} for servo ${id}: ${e.message}`);
      }
    }
  }

  scale(value) {
    const midpoint = (this.max_us + this.min_us) / 2;
    const range = (this.max_us - this.min_us) / 2;
    return Math.round(midpoint + range * value);
  }

  setServoPWM(name, value) {
    const id = this.channelMap[name];
    const gpio = this.outputs[id];

    if (!gpio) {
      console.error(`Invalid servo name or GPIO not initialized: ${name}`);
      return;
    }

    const pulseWidth = this.scale(value);
    gpio.servoWrite(pulseWidth);
    console.log(`PWM set: ${name} → ${pulseWidth}us on GPIO${this.pins[id]}`);
  }
}

module.exports = PWMPigpioServo;

