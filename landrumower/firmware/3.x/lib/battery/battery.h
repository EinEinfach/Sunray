#include <Arduino.h>

class Battery
{
public:
    Battery(uint8_t pinPowerSwitch);
    float voltage;
    float chgVoltage;
    float chgCurrent;
    float temperature;
};