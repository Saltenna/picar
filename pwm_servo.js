// pwm_servo.js
const fs = require('fs');

function detectPiModel() {
  try {
    const model = fs.readFileSync('/proc/device-tree/model', 'utf8').toLowerCase();
    if (model.includes('pi 5') || model.includes('pi5')) return 5;
    if (model.includes('pi 4') || model.includes('pi4')) return 4;
    if (model.includes('pi 3') || model.includes('pi3')) return 3;
  } catch (_) {}
  return null;
}

// Map Pi model to best PWM driver (when not using mavproxy)
const piDriverMap = {
  5: 'sysfs',      // Pi 5: hardware PWM via sysfs (pigpio not supported)
  4: 'pigpion',    // Pi 4: native pigpio C library
  3: 'pigpion',    // Pi 3: native pigpio C library
};

function PWMDriver(config) {
  let method = config.pwm_method;

  // mavproxy is platform-independent — no override needed
  if (method !== 'mavproxy') {
    const piModel = detectPiModel();
    if (piModel) {
      const autoMethod = piDriverMap[piModel];
      if (method !== autoMethod) {
        console.log(`Pi ${piModel} detected — overriding pwm_method "${method}" → "${autoMethod}"`);
        method = autoMethod;
      } else {
        console.log(`Pi ${piModel} detected — pwm_method "${method}" is compatible`);
      }
    } else {
      console.log(`Could not detect Pi model, using configured pwm_method "${method}"`);
    }
  } else {
    console.log(`Using MAVProxy PWM driver (platform-independent)`);
  }

  try {
    const Module = require(`./pwm_${method}_servo.js`);
    return new Module(config);
  } catch (err) {
    console.error(`Failed to load PWM driver for method "${method}": ${err.message}`);
    process.exit(1);
  }
}

module.exports = PWMDriver;

