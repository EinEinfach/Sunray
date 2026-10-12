# Landrumower

Landrumower is a self-built robot mower running Sunray. A Raspberry Pi runs Sunray as a Linux program, and a Raspberry Pi Pico on a custom board drives the motors and reads the sensors.

```
Raspberry Pi 4 (Sunray)  <-- UART, 115200 baud, AT protocol -->  Pico (this firmware)
                                                                   |- 3 motor drivers (JYQD), encoders
                                                                   |- 4 x INA226 current sensors (I2C)
                                                                   |- LCD (I2C), buzzer
                                                                   |- bumper, lift, stop button, rain sensor
```

## Folders

| Folder | Content |
|---|---|
| `firmware/4.x` | Pico firmware in C++ (PlatformIO). Current development, not feature complete yet. |
| `firmware/3.x` | Earlier C++ attempt, never finished. Reference only. |
| `firmware/2.x` | MicroPython firmware with all features, too slow for motor control. Reference only. |
| `firmware/1.x` | First MicroPython firmware. Reference only. |
| `schematics` | KiCad project of the board. Known board issues are listed in `findings.txt`. |
| `scripts` | `deploy.sh` uploads the MicroPython firmware (1.x, 2.x) with `mpremote`. |
| `sim` | `sunray_sim.py` plays Sunray for firmware 4.x over USB. `sim_sunray_comm.py` is the older script for 2.x/3.x. |
| `config.h` | Sunray configuration for Landrumower. |

## Pico firmware 4.x

### Requirements

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html), for example `pip install platformio`
- A USB cable with data lines for flashing. Charge-only cables do not work.

PlatformIO downloads the toolchain and the test framework on first use.

If `pio` is not found, it is not on your `PATH`. When PlatformIO is installed in a Python virtual environment, activate that environment first, for example for a `.venv` in the repository root:

```
source .venv/bin/activate         # bash, zsh
source .venv/bin/activate.fish    # fish
```

Or call it by its path, for example `../../../.venv/bin/pio` from the firmware folder.

All commands below are run in `landrumower/firmware/4.x`.

### Run the unit tests

The tests run on the development machine. No Pico is needed.

```
pio test -e native
```

Run the tests of one feature only:

```
pio test -e native -f test_protocol
```

### Build

```
pio run -e pico        # real hardware
pio run -e pico_sim    # Pico with simulated motors and current sensors
```

The result is `.pio/build/<environment>/firmware.uf2`.

`pico_sim` is meant for a bench setup without motors and drivers. The three motors with their encoders are replaced by a simple model inside the firmware, and the motor pins are not touched. Everything else runs the same code as on the mower, including the real LCD and buzzer.

### Flash

Either let PlatformIO upload:

```
pio run -e pico -t upload        # or -e pico_sim
```

Or copy the file by hand: hold BOOTSEL while plugging in the Pico, then copy `firmware.uf2` to the drive `RPI-RP2` that appears.

### Talk to the firmware

Open the USB console:

```
pio device monitor -b 115200
```

Type a command and press Enter. On USB the checksum may be omitted:

```
AT+V
```

The Pico answers with name and version, followed by the checksum:

```
V,Landrumower RPI Pico,4.0.0,0x..
```

While a USB console is open, the firmware also prints the state of the control loop every 100 ms (`TELEMETRY_PERIOD_MS` in `include/config.h`):

```
D,<ms>,<demand>,<speed>,<drive>,<brake>,  ... right wheel
       <demand>,<speed>,<drive>,<brake>,  ... left wheel
       <demand>,<speed>,<drive>,<brake>,  ... mow motor
       <longest control period in us>
```

It is one line. Demand and speed are in mm/s for the wheels (PWM value and impulses per second for the mow motor), drive and brake are duty cycles from 0 to 1, and drive is negative when driving backwards.

Once per second (`INFO_PERIOD_MS`) it prints the mower state, whether the LCD answers on I2C, and whether the buzzer is switched on:

```
I,state=mow,lcd=1,buzzer=0
```

### Bench test without mower

`sim/sunray_sim.py` plays the role of Sunray over USB. It sends `AT+M` every 20 ms and `AT+S` every 500 ms, follows a profile of demands and evaluates answers and telemetry. While a demand is not zero it reports the Sunray state "mow", otherwise "idle", so LCD and buzzer react as well. It needs `pyserial`, which comes with PlatformIO.

Flash `pico_sim`, then from the repository root:

```
python landrumower/sim/sunray_sim.py                             # 300, -300, 0 mm/s
python landrumower/sim/sunray_sim.py --profile 150:3,400:3,0:2   # demand:seconds
python landrumower/sim/sunray_sim.py --pwm --profile 128:3,0:2   # PWM pass-through
python landrumower/sim/sunray_sim.py --csv run.csv               # store the telemetry
```

Example output:

```
firmware: Landrumower RPI Pico 4.0.0
speed profile 300:3,-300:4,0:2
  demand   300: right  300.0  left  300.0  drive +0.42  brake 0.00  settled after 0.10 s
  demand  -300: right  300.0  left  300.0  drive -0.42  brake 0.00  settled after 0.70 s  ticks +546/+546
  demand     0: right    0.0  left    0.0  drive -0.00  brake 1.00  settled after 0.60 s  ticks +31/+31
answers: AT+M 450 of 450, AT+S 18 of 18, crc errors 0, unreadable lines 0
control period: longest 5495 us, typical longest per telemetry line 5008 us
lcd found, buzzer switched on in 2 of 10 info lines, states: idle > mow > idle
```

The measured speed has no sign, because the encoders have no direction.

### LCD and buzzer

Both run on the second core, so the slow I2C transfer to the display does not disturb the motor control.

- The LCD (16x2, PCF8574 backpack, address `LCD_ADDRESS`) shows name and version after power-on, then the Sunray state from `AT+S` in the first line. The second line is reserved for battery voltage and current. A display that is missing or unplugged is searched again every `LCD_RETRY_MS`.
- The buzzer signals each change of the state once: one long tone for idle, charge, dock and the GPS states, three tones for mow, error and escape, a series of short tones for IMU calibration.
- The buzzer pin is only switched on and off. GP26 shares its PWM slice with the mow motor PWM on GP11, so a PWM frequency for the buzzer would change the PWM frequency of the mow motor.

### Motor control

The settings are in `include/config.h`.

- `PICO_MOTOR_CONTROL true`: the Pico controls the wheel speed. Sunray sends the speed in mm/s in the last two fields of `AT+M`. If these fields are missing, the PWM values are used.
- `PICO_MOTOR_CONTROL false`: Sunray controls, the Pico passes the PWM values through.
- The mow motor always uses its PWM value.
- `MOTOR_KP`, `MOTOR_KI`, `MOTOR_KD`: controller gains, output is duty (0..1) per mm/s. `AT+S,<state>,<kp>,<ki>,<kd>` replaces them at runtime.
- A motor is stopped before it is reversed, and all motors stop if no `AT+M` arrives for `MOTOR_COMMAND_TIMEOUT_S`.
- `BRAKE_PWM`, `BRAKE_MIN_DUTY`, `BRAKE_MAX_DUTY`: braking with PWM on the brake input of the drivers. This is untested on the real drivers. The usable duty range has to be measured first. With `BRAKE_PWM false` the brake is only switched on and off.

### Protocol

Sunray sends requests, the Pico answers each one. Every line ends with `,0x<crc>` and `\r\n`. The CRC is the sum of all bytes before that last comma, modulo 256, as two hex digits.

| Request | Meaning | Answer |
|---|---|---|
| `AT+V` | version | `V,<name>,<version>` |
| `AT+M,<rightPwm>,<leftPwm>,<mowPwm>[,<rightSpeed>,<leftSpeed>]` | motor command, PWM -255..255, speed in mm/s | `M,<rightTicks>,<leftTicks>,<mowTicks>,<chargeVoltage>,<bumper>,<lift>,<stopButton>` |
| `AT+S[,<state>[,<kp>,<ki>,<kd>]]` | summary request with Sunray state and optional new PID gains | `S,<batVoltage>,<chgVoltage>,<chgCurrent>,<lift>,<bumper>,<rain>,<motorFault>,<mowCurrent>,<leftCurrent>,<rightCurrent>,<batTemp>` |
| `AT+R` | reset motor faults | `R` |
| `AT+Y3` | shutdown | `Y3` |
| `AT+E` | motor control test (developer) | |
| `AT+Y` | provoke the watchdog (developer) | `Y` |

On the UART to the Pi the CRC is required. On USB a line without CRC is accepted, but a CRC that is present must be correct.

In 4.x only `AT+V`, `AT+M` and `AT+S` are answered so far. `AT+M` drives the motors and returns the encoder ticks. `AT+S` takes over state and gains. Charge voltage, sensors and everything in the `AT+S` answer are still zero.

### Add a feature

Each feature is one folder in `lib/` with its tests in a folder of the same name in `test/`, prefixed with `test_`:

```
lib/pid/pid.h, pid.cpp
test/test_pid/test_pid.cpp
```

Keep logic free of Arduino includes so it can be tested on the development machine. Code that needs the hardware goes into separate files wrapped in `#ifdef ARDUINO`, behind a small interface that a simulation can implement as well (see `MotorDriver` and `EncoderSource`).

## Older firmware

- **3.x:** `pio run -t upload` in `firmware/3.x`.
- **1.x and 2.x:** run `../../scripts/deploy.sh` in the firmware folder. It expects a Python environment with `mpremote` in `~/pico/.venv` and the Pico on `/dev/ttyACM1`.

## Sunray on the Raspberry Pi

### Installation

1. Perform a fresh installation of Raspberry Pi OS on your Raspberry Pi.
2. Install `cmake` and `libbluetooth-dev`.
3. Enable the hardware UART in `raspi-config`.
4. Run `sudo nano /boot/config.txt` and add this line to activate the software-based I2C driver:

       dtoverlay=i2c-gpio,bus=1,i2c_gpio_sda=2,i2c_gpio_scl=3

5. Clone the repository and switch to the branch:

       git clone https://github.com/EinEinfach/Sunray.git
       cd Sunray
       git checkout landrumower-linux

6. Compile Sunray with the Landrumower configuration:

       cd alfred/build
       cmake -D CONFIG_FILE=$PWD/../../landrumower/config.h ..
       make

   The build copies the chosen configuration to `sunray/config.h`. Without `CONFIG_FILE` it uses `alfred/config.h`.

7. Create `/etc/systemd/system/sunray.service` with this content. Adjust the path in `ExecStart` to your user:

       [Unit]
       Description=sunray
       [Service]
       User=root
       Group=root
       Type=simple
       Restart=always
       StandardInput=tty
       StandardOutput=journal
       TTYPath=/dev/tty12
       ExecStart=/usr/bin/stdbuf -oL -eL /home/lex/Sunray/alfred/build/sunray
       [Install]
       WantedBy=multi-user.target

8. Reload systemd and start the service:

       sudo systemctl daemon-reload
       sudo systemctl enable --now sunray

### Run and watch

```
sudo systemctl restart sunray     # restart after a new build
sudo systemctl stop sunray
journalctl -u sunray -f           # follow the log
```

Sunray talks to the Pico on `/dev/ttyS0` with 115200 baud (`SERIAL_ROBOT_PATH` and `ROBOT_BAUDRATE` in `config.h`).
