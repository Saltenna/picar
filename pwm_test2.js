const { Gpio } = require('pigpio');

const THROTTLE_PIN = 17;
const STEERING_PIN = 18;

// Initialize GPIOs
const throttle = new Gpio(THROTTLE_PIN, { mode: Gpio.OUTPUT });
const steering = new Gpio(STEERING_PIN, { mode: Gpio.OUTPUT });

// Helper function to convert from 0.105–0.175 range to 0–255
function pwmValueToDutyCycle(pwmValue) {
  return Math.round((pwmValue - 0.105) * (255 / (0.175 - 0.105)));
}

// Function to set servo position
function setServo(pin, pwmValue) {
  const duty = pwmValueToDutyCycle(pwmValue);
  pin.pwmWrite(Math.max(0, Math.min(255, duty)));
}

// Example: Sweep throttle and steering
let pos = 0.105;
let dir = 0.002;

console.log('Sweeping PWM on pins 17 (throttle) and 18 (steering)...');

const interval = setInterval(() => {
  setServo(throttle, pos);
  setServo(steering, pos);

  pos += dir;
  if (pos >= 0.175 || pos <= 0.105) dir = -dir;

  console.log(`PWM set to: ${pos.toFixed(3)}`);
}, 100);

// Stop on Ctrl+C
process.on('SIGINT', () => {
  console.log('\nStopping and resetting pins to neutral (0.14)');
  setServo(throttle, 0.14);
  setServo(steering, 0.14);
  clearInterval(interval);
  process.exit();
});

