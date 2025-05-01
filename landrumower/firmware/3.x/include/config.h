#ifndef CONFIG_H
#define CONFIG_H

#define WATCHDOG false
#define DEBUG false
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

#define VERNR "2.5.1"
#define VER "Landrumower RPI Pico " VERNR

// define pins
#define I2C0_SDA 16
#define I2C0_SCL 17
#define I2C1_SDA 14
#define I2C1_SCL 15
#define PIN_BUZZER 26

#endif // CONFIG_H