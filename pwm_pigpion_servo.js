// pwm_pigpion_servo.js
// This pwm backend drives the pwm on raspberry pi 3 and 4 via the 
// native or raw pwm driver. it avoids the systemd pigpiod daemon
// it should not be necessary but may require root permissions to drive it
//

const pigpio = require('pigpio');
const Gpio = pigpio.Gpio;

const THROTTLE = 0;
const STEERING = 1;

class PWMServoPigpio {
  constructor(config) {
    this.period_us = config.pwm_period_us;
    this.min_us = config.pwm_min_us;
    this.max_us = config.pwm_max_us;

    this.pins = {
      [THROTTLE]: config.throttle_gpio,
      [STEERING]: config.steering_gpio
    };

    this.outputs = {};
    for (const [idStr, pin] of Object.entries(this.pins)) {
      const id = Number(idStr);
      this.outputs[id] = new Gpio(pin, { mode: Gpio.OUTPUT });
      console.log(`Configured GPIO${pin} for PWM (servo ${id})`);
    }
  }

  scale(value) {
    const midpoint = (this.max_us + this.min_us) / 2;
    const range = (this.max_us - this.min_us) / 2;
    return Math.round(midpoint + range * value);
  }

  setServoPWM(name, value) {
    const channelMap = { throttle: THROTTLE, steering: STEERING };
    const id = channelMap[name];
    const gpio = this.outputs[id];
    if (!gpio) {
      console.error(`Invalid servo id: ${id}`);
      return;
    }

    const pulseWidth = this.scale(value);
    gpio.servoWrite(pulseWidth);
    //if (id)
    //console.log(`PWM (pigpio): ${name} → ${pulseWidth}μs`);
  }
}

module.exports = PWMServoPigpio;

