# Installation
1. Perform a fresh installation of Raspbian OS on your Raspberry Pi.
2. Install cmake, libbluetooth-dev
3. Enable the hardware UART in raspi-config.
4. Run 'sudo nano /boot/config.txt' and add this line to activate the software-based I2C driver: dtoverlay=i2c-gpio,bus=1,i2c_gpio_sda=2,i2c_gpio_scl=3
5. Clone https://github.com/EinEinfach/Sunray.git repository
6. Switch to landrumower-linux branch
7. Compile sunray fw
8. Create sunray.service file in /etc/systemd/system and add this lines:

    [Unit]
    Description=sunray
    [Service]
    #your username!
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

9. Reload systemd
