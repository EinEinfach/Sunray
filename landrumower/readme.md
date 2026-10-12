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
| `sim` | Script that sends motor commands to the Pico over USB. Written for 2.x/3.x. |
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

`pico_sim` is meant for a bench setup without motors, drivers and INA226. The simulation itself is not implemented yet, so both environments currently behave the same.

### Flash

Either let PlatformIO upload:

```
pio run -e pico -t upload
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

In 4.x only `AT+V`, `AT+M` and `AT+S` are answered so far, and `AT+M` and `AT+S` return empty reports.

### Add a feature

Each feature is one folder in `lib/` with its tests in a folder of the same name in `test/`, prefixed with `test_`:

```
lib/pid/pid.h, pid.cpp
test/test_pid/test_pid.cpp
```

Keep logic free of Arduino includes so it can be tested on the development machine.

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
