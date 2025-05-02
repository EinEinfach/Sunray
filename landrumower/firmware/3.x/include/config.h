#ifndef CONFIG_H
#define CONFIG_H

#define VERNR "3.0.0"
#define VER "Landrumower RPI Pico " 

#define WATCHDOG false
#define DEBUG true
#define INFO true
#define INFOTIME 10000
#define HIL false
#define PICOMOTORCONTROL true

#define KP 8.0f
#define KI 0.1f
#define KD 0.005f

#define OVERLOADCURRENT_GEAR 10
#define OVERLOADCURRENT_MOW 10
#define CRITICALVOLTAGE 17

#define FREQ 20000
#define FREQ_MOW 10000
#define CURRENTFACTOR 10

#define INABATADRESS 0x40
#define INAMOWADRESS 0x41
#define INALEFTADRESS 0x44
#define INARIGHTADRESS 0x45

#define LCD true
#define LCDADRESS 0x27
#define LCD_NUM_ROWS 2
#define LCD_NUM_COLUMNS 16

#define BUZZER_LOUDNESS 10 //(0->255)
#define RAINSENSOR_THRESHOLD 32000 //(0->65535)

/******************************* from here is actually no individuell configuration is needed ********************************/
// uart
#define UART Serial1
#define USB Serial
#define UART_BAUDRATE 115200
#define USB_BAUDRATE 115200

// i2c
#define I2C0_CLOCK 400000
#define I2C1_CLOCK 400000

// define pins
// uart
#define UART0_TX 0
#define UART0_RX 1
// i2c
#define I2C0_SDA 16
#define I2C0_SCL 17
#define I2C1_SDA 14
#define I2C1_SCL 15
#define PIN_BUZZER 26
// motor right
#define RIGHT_IMP 2
#define RIGHT_PWM 3
#define RIGHT_DIR 4
#define RIGHT_BRAKE 5
// motor left
#define LEFT_IMP 6
#define LEFT_PWM 7
#define LEFT_DIR 8
#define LEFT_BRAKE 9
// motor mow
#define MOW_IMP 10
#define MOW_PWM 11
#define MOW_DIR 12
#define MOW_BRAKE 13
// bumper, lift, stop, rain
#define BUMPER_X 18
#define BUMPER_Y 19
#define LIFT 20
#define STOP 21
#define RAIN 28
// battery
#define POWER_SWITCH 22

#endif // CONFIG_H