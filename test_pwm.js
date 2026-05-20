// test_pwm.js
const fs = require('fs');
const path = require('path');
const PWMDriver = require('./pwm_servo');

// Load config
const configPath = path.join(__dirname, 'picar-cfg.json');
let config;
try {
  config = JSON.parse(fs.readFileSync(configPath));
} catch (err) {
  console.error(`Failed to read config file at ${configPath}:`, err);
  process.exit(1);
}

const pwm = PWMDriver(config);

const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));

async function sweep(id, label) {
  console.log(`Sweeping ${label} (id=${id})...`);

  for (let v = -1; v <= 1.01; v += 0.1) {
    pwm.setServoPWM(id, v);
    console.log(`${label}: ${v.toFixed(1)}`);
    await sleep(200);
  }

  for (let v = 1; v >= -1.01; v -= 0.1) {
    pwm.setServoPWM(id, v);
    console.log(`${label}: ${v.toFixed(1)}`);
    await sleep(200);
  }

  pwm.setServoPWM(id, 0);
  console.log(`${label} centered.\n`);
}

async function main() {
  console.log('--- PWM Servo Test ---');
  await sweep('steering', 'Steering');
  await sweep('throttle', 'Throttle');
}

main().catch(console.error);

