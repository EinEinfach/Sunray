#include <Arduino.h>
#include <LCD_I2C.h>
#include "buzzer.h"

class PicoDriver {
    public:
        PicoDriver();
        void setup();
        void printLcd(String message);
        Buzzer buzzer;
    private:
        String printedMessage;
        LCD_I2C lcd;
};