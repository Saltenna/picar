// pwm_libgpiod_servo.js
// author michael sundius
// this uses libgpiod for the raspberry pi 5 choose this one only for pi 5
//

const { execSync } = require('child_process');

const THROTTLE = 0;
const STEERING = 1;

class PWMServoGPIOD {
  constructor(config) {
    this.period_us = config.pwm_period_us;
    this.min_us = config.pwm_min_us;
    this.max_us = config.pwm_max_us;
    this.chip = config.pwm_chip || 'gpiochip0';

    this.pins = {
      [THROTTLE]: config.throttle_gpio,
      [STEERING]: config.steering_gpio
    };

    this.lastPWM = {
      [THROTTLE]: 0,
      [STEERING]: 0
    };

    this.intervals = {};
    this.channelMap = {
      throttle: THROTTLE,
      steering: STEERING
    };
  }

  scale(value) {
    const midpoint = (this.max_us + this.min_us) / 2;
    const range = (this.max_us - this.min_us) / 2;
    return Math.round(midpoint + range * value);
  }

  setServoPWM(name, value) {
    const id = this.channelMap[name];
    if (id === undefined || !(id in this.pins)) {
      console.warn(`Invalid servo id: ${name}`);
      return;
    }

    this.lastPWM[id] = value;
    this.startPWM(id, this.pins[id]);
  }

  startPWM(id, pin) {
    if (this.intervals[id]) clearInterval(this.intervals[id]);

    this.intervals[id] = setInterval(() => {
      const pulseWidthUs = this.scale(this.lastPWM[id]);

      try {
        execSync(`gpioset ${this.chip} ${pin}=1`);
      } catch (err) {
        console.error(`Failed to set pin ${pin} HIGH: ${err.message}`);
      }

      setTimeout(() => {
        try {
          execSync(`gpioset ${this.chip} ${pin}=0`);
        } catch (err) {
          console.error(`Failed to set pin ${pin} LOW: ${err.message}`);
        }
      }, pulseWidthUs / 1000); // µs → ms
    }, this.period_us / 1000); // µs → ms
  }
}

module.exports = PWMServoGPIOD;

