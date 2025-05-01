#include <Arduino.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include "config.h"
#include "picodriver.h"

PicoDriver::PicoDriver() 
    : buzzer(PIN_BUZZER), 
    lcd(LCD_I2C(LCDADRESS, LCD_NUM_COLUMNS, LCD_NUM_ROWS)) {
    // Initialize
    buzzer.stopPlaying();
}

void PicoDriver::setup() {
    // configure i2c0 (INAs)
    Wire.setSDA(I2C0_SDA);
    Wire.setSCL(I2C0_SCL);
    Wire.setClock(400000);
    Wire.setTimeout(2);
    Wire.begin();
    // configuration of i2c1 is done by LCD_I2C library
    printedMessage = "";
    if (LCD) {
        lcd.begin();
        lcd.backlight();
    }
}

void PicoDriver::printLcd(String message) {
    if (printedMessage != message) {
        lcd.print(message);
        printedMessage = message;
    }
}