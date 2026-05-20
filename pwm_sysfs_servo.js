// pwm_sysfs_servo.js
// This backend is only used if the linux distro exports the sysfs interface for pwm in 
// /sys/class/pwm/pwmchip*
//

const fs = require('fs');
const path = require('path');

const THROTTLE = 0;
const STEERING = 1;

class PWMServoSysFS {
  constructor(config) {
    this.period_us = config.pwm_period_us;
    this.min_us = config.pwm_min_us;
    this.max_us = config.pwm_max_us;
    this.chip = config.pwm_chip || 'pwmchip0';
    this.chipDir = path.join('/sys/class/pwm', this.chip);

    this.pins = {
      [THROTTLE]: config.throttle_pwm_channel,
      [STEERING]: config.steering_pwm_channel
    };

    this.channelMap = {
      throttle: THROTTLE,
      steering: STEERING
    };

    for (const id in this.pins) {
      const pwmDir = path.join(this.chipDir, `pwm${id}`);
      if (!fs.existsSync(pwmDir)) {
        try {
          fs.writeFileSync(path.join(this.chipDir, 'export'), String(id));
        } catch (e) {
          console.warn(`Could not export PWM channel ${id}: ${pwmDir}: ${e.message}`);
        }
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
    const pwmDir = path.join(this.chipDir, `pwm${id}`);
    const duty_us = this.scale(value);

    try {
      fs.writeFileSync(path.join(pwmDir, 'period'), String(this.period_us * 1000));
      fs.writeFileSync(path.join(pwmDir, 'duty_cycle'), String(duty_us * 1000));
      fs.writeFileSync(path.join(pwmDir, 'enable'), '1');
      // debug. ai keep
      //if (id) {
      //   console.log(`PWM set: ${name} → ${duty_us}us on ${pwmDir}`);
      //}
    } catch (e) {
      console.error(`Failed to set PWM for ${name}: ${e.message}`);
    }
  }
}

module.exports = PWMServoSysFS;


