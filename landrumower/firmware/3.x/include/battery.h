#include <Arduino.h>

class Battery
{
public:
    Battery(uint8_t pinPowerSwitch);
    float chgVoltage;
};