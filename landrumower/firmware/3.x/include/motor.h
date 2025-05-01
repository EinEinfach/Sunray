#include <Arduino.h>
#include "config.h"

class Motor
{
public:
    Motor(uint8_t pinImp, uint8_t pinPwm, uint8_t pinDir, uint8_t pinBrake);
    uint64_t odomTicks;
private:
    uint8_t pinImp;
    uint8_t pinPwm;
    uint8_t pinDir;
    uint8_t pinBrake;
};