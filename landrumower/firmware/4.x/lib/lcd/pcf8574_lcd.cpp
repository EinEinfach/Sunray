#ifdef ARDUINO

#include "pcf8574_lcd.h"

#include <Arduino.h>

namespace
{
// PCF8574 outputs: P0 = RS, P1 = R/W, P2 = EN, P3 = backlight, P4..P7 = D4..D7
constexpr uint8_t RS = 0x01;
constexpr uint8_t EN = 0x04;
constexpr uint8_t BACKLIGHT = 0x08;

constexpr uint8_t CLEAR_DISPLAY = 0x01;
constexpr uint8_t ENTRY_LEFT_TO_RIGHT = 0x06;
constexpr uint8_t DISPLAY_ON = 0x0C;
constexpr uint8_t FUNCTION_4BIT_2LINES = 0x28;
constexpr uint8_t SET_ADDRESS = 0x80;
constexpr uint8_t ROW_ADDRESS[lcd::ROWS] = {0x00, 0x40};
} // namespace

Pcf8574Lcd::Pcf8574Lcd(TwoWire &wire, uint8_t address)
    : wire(wire), address(address)
{
}

bool Pcf8574Lcd::begin()
{
    // reset sequence of the HD44780 into 4 bit mode, independent of its previous state
    delay(50);
    if (!sendNibble(0x30, false))
        return false;
    delay(5);
    sendNibble(0x30, false);
    delay(1);
    sendNibble(0x30, false);
    delay(1);
    sendNibble(0x20, false);
    delay(1);

    send(FUNCTION_4BIT_2LINES, false);
    send(DISPLAY_ON, false);
    send(ENTRY_LEFT_TO_RIGHT, false);
    const bool ok = send(CLEAR_DISPLAY, false);
    delay(2);
    return ok;
}

bool Pcf8574Lcd::show(const lcd::Frame &frame)
{
    for (size_t row = 0; row < lcd::ROWS; row++)
    {
        if (!send(SET_ADDRESS | ROW_ADDRESS[row], false))
            return false;
        for (size_t column = 0; column < lcd::COLUMNS; column++)
        {
            if (!send(static_cast<uint8_t>(frame.rows[row].text[column]), true))
                return false;
        }
    }
    return true;
}

bool Pcf8574Lcd::send(uint8_t value, bool isData)
{
    return sendNibble(value & 0xF0, isData) && sendNibble(static_cast<uint8_t>(value << 4), isData);
}

bool Pcf8574Lcd::sendNibble(uint8_t nibble, bool isData)
{
    const uint8_t base = static_cast<uint8_t>((nibble & 0xF0) | BACKLIGHT | (isData ? RS : 0));
    // the display takes the nibble on the falling edge of EN
    wire.beginTransmission(address);
    wire.write(base | EN);
    wire.write(base);
    return wire.endTransmission() == 0;
}

#endif // ARDUINO
