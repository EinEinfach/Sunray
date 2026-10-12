// HD44780 character display behind a PCF8574 I2C backpack, 4 bit mode.

#ifndef PCF8574_LCD_H
#define PCF8574_LCD_H

#ifdef ARDUINO

#include <Wire.h>
#include "lcd.h"

class Pcf8574Lcd
{
public:
    Pcf8574Lcd(TwoWire &wire, uint8_t address);

    // Initialises the display. Returns false if it does not answer.
    bool begin();
    // Writes all rows. Returns false if the display does not answer.
    bool show(const lcd::Frame &frame);

private:
    bool send(uint8_t value, bool isData);
    bool sendNibble(uint8_t nibble, bool isData);

    TwoWire &wire;
    uint8_t address;
};

#endif // ARDUINO

#endif // PCF8574_LCD_H
