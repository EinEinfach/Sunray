#include <Arduino.h>
#include "battery.h"

Battery::Battery(uint8_t pinPowerSwitch)
{
    voltage = 0.0;
    chgVoltage = 0.0;
    chgCurrent = 0.0;
    temperature = 0.0;
}
