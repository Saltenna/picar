# picar

## Overview
This is a modernized web-controlled RC car platform running on a Raspberry Pi 5 with Bookworm OS and Node.js v18.19.0. It allows real-time control of an RC car via a browser, supporting both desktop and mobile control interfaces. The mobile interface uses the device's IMU for tilt-based control.

The car is equipped with:
- A USB webcam for video streaming
- Throttle and steering servos controlled via PWM
- A local HTTPS server that hosts the UI and control API

Originally derived from the original Pi RC car project by [lawsonkeith](https://github.com/XXX/picar), this version:
- Replaces `pi-blaster` with an interface allowing direct `sysfs` or `libgpiod` control for PWM, soon to be mavlink and ardupilot
- Uses current node.js and npm practices
- Supports modern mobile orientation APIs

## Hardware Setup
- **Platform**: Raspberry Pi 5 or 4B+
- **Servos**: Controlled via GPIO PWM using sysfs (soon to be replaced with `libgpiod`)
- **Camera**: USB webcam accessible via `/dev/video0`
- **Network**: Typically configured to use smartphone hotspot for local control

## Electrical
This project is based on [shaunuk/picar] but replaces the servo board with a soft PWM driver on the Pi’s GPIO pins. I've provided the option to use either an isolated supply for the Pi or to use the supply off the ESC. The latter is the most elegant solution but has issues with the motor pulling the battery voltage down and causing the Pi to reset.

There's a video of the project here:

<dummy video>

### Pi Power Supply
When it comes to powering the Pi it is necessary to have a stable 5V power supply otherwise the Pi will reset. I've provided 2 methods of powering the Pi; one using a 7.2V AA battery pack and a 5V USB battery or linear regulator to give a clean 5V supply and one that uses the ESC power supply and uses a software algorithm to stop the motor accidentally resetting the Pi.

There are a number of RC car electrical setups but my example uses an ESC and receiver with battery eliminator circuit. The electronics supply normally comes from the ESC and powers the receiver and steering servo with 5V. The receiver normally receives commands from the radio controller then sends them to the ESC (throttle) and steering servo (steering). These commands fall within the 0-5V supplied by the ESC.

### Servo Signal Levels
We'll be using PWM to control the servos which will be capable of driving 0–3.3V in 3.3mV steps; it is therefore necessary first to measure what your servo command signal voltage levels are and check they fall within this range. You can do this with a multi-meter connected to the receiver pins. On my car speed and steering both use 3-pin headers which are wired:

- Gnd - Black
- Power - Red
- Throttle / Steer - Orange or White

For my car the throttle voltages were:
- Full Fwd:     0.19V
- Idle:         0.28V
- Full Reverse: 0.36V

The steering voltages were:
- Full Left:    0.19V
- Fwds:         0.28V
- Full Right:   0.355V

### Pi with Isolated Supply
Once you are happy about how you are going to power your Pi and that the GPIO are up to the job you can start thinking about wiring it up.

![](https://github.com/XXX/picar/raw/master/media/picar_scematic.PNG)

For the 3-pin headers I used 2.54mm PCB header and soldered the wires direct. I then used superglue to stop the pins moving about. You'll also need heat-shrink or equivalent to cover over the solder joints.

To power the Pi I chopped a micro USB cable and used the black and red wires from that as my 5V supply. For my linear power supply I used an LM7805 circuit and put a heatsink on it to keep it nice and cool. I'd recommend using a different LDO regulator if you have one though; this circuit will stop working at 7V; a LM2940CT will regulate down to 5.5V and would be a much better option.

![](https://github.com/XXX/picar/raw/master/media/reg cct.PNG)

I then attached a PP3 battery clip and 6xAA pack with a PP3 connector on it. I've also used a 26w header socket to attach to the Raspberry Pi GPIO lines; I like this method as it means it's hard to mis-wire when re-connecting plus you can quickly remove your Pi as required.

I've covered my regulator circuit in heatshrink to keep it protected. You can see I've also got some protection on my header pins in case a wire falls off.

![](https://github.com/XXX/picar/raw/master/media/DSC1499.jpg)
![](https://github.com/XXX/picar/raw/master/media/DSC_0219.jpg)
![](https://github.com/XXX/picar/raw/master/media/DSC_0220.jpg)
![](https://github.com/XXX/picar/raw/master/media/DSC_0221.jpg)

### Pi using ESC Supply
If you opt for this approach the wiring is much simpler; also there's a lot more room in the car. The only issue is you may have to play around with the motor demand rate-of-change limiting algorithm in `app.js` to get to a point where your Pi doesn't keep resetting because of voltage drop caused by the motor loading the battery. You can control this problem quite effectively by controlling the rate of change of the speed demand that's allowed to be sent to the ESC from the Pi. The amount of rate limiting you require will depend on:

- Motor power (stock being best)
- Battery technology (NiMH better than NiCad)
- Battery capacity and condition
- Drive type (2WD better than 4WD)
- Efficiency of ESC power supply circuitry
- How the ESC reverse works (brake → reverse can cause a big demand change)
- Running surface (grass being worst)

![](https://github.com/XXX/picar/raw/master/media/picar_scematic_nobat.PNG)
![](https://github.com/XXX/picar/raw/master/media/DSCF1517.jpg)
![](https://github.com/XXX/picar/raw/master/media/DSCF1515.jpg)

## Software Setup

### Install Node.js and Dependencies
Node.js and npm are available through the official Raspberry Pi OS Bookworm repository:

```bash
sudo apt update
sudo apt install nodejs npm ffmpeg
```

### Get the App
Clone the project into your working directory:
```bash
cd /home/pi
mkdir picar && cd picar
# Replace with your fork or repo URL
git clone https://github.com/XXX/picar .
```

### Install Required Node Packages
```bash
npm install socket.io node-static
```

### PWM Support for Raspberry Pi 5

To allow non-root users to access PWM devices via sysfs, create a udev rules file at `/etc/udev/rules.d/99-pwm-permissions.rules`:

```bash
sudo nano /etc/udev/rules.d/99-pwm-permissions.rules
```

Insert the following:
```udev
SUBSYSTEM=="pwm", KERNEL=="pwmchip*", MODE="0777", GROUP="gpio"
SUBSYSTEM=="pwm", KERNEL=="pwm*", MODE="0777", GROUP="gpio"
```

Then reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
PWM output to the servos is handled through `pwm_servo.js`, which uses the `/sys/class/pwm` interface. On Raspberry Pi 5, this works with kernel support, but may require enabling the `pwm` overlay in `/boot/firmware/config.txt`:

```ini
# Enable PWM channels
dtoverlay=pwm-2chan
```

Ensure your user has access to `/sys/class/pwm` or run the app as root.

Be sure you are a member of the gpio group

```bash
sudo usermod -aG gpio $USER
```
or just add your username to the end of the "gpio" line in /etc/group

```bash
sudo vi /etc/group # comma separated list of user names
```
AND change the group of /dev/gpio

```bash
sudo chgrp gpio /dev/pigpio
```


### Network Configuration
To use your smartphone as a hotspot:

- On your iPhone, enable the Personal Hotspot.
- On the Raspberry Pi:
  1. Use the GUI or `nmtui` (NetworkManager) to scan for and connect to the iPhone hotspot.
  2. Save the connection for automatic reconnect.
  3. Ensure the Pi receives an IP address and has internet access through the hotspot.

This allows your mobile device to connect to the Pi's services over a local secure HTTPS connection.

### Running the Application
```bash
sudo node app.js
```
Open a browser and go to:
```
https://<raspberry-pi-ip>:8443/socket.html
```
To view the camera stream:
```
https://<raspberry-pi-ip>:8081/stream.mjpg
```

### Configure Pi to Run App on Boot
Create a systemd service file `/etc/systemd/system/picar.service`:

```ini
[Unit]
Description=PiCar Control Server
After=network.target

[Service]
ExecStart=/usr/bin/node /home/pi/picar/app.js
WorkingDirectory=/home/pi/picar
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi
Environment=NODE_ENV=production

[Install]
WantedBy=multi-user.target
```

Then enable it:
```bash
sudo systemctl daemon-reexec
sudo systemctl enable picar
sudo systemctl start picar
```

The web server and stream will now automatically start when the Pi boots.

