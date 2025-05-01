#include <Arduino.h>
#include "battery.h"

Battery::Battery(uint8_t pinPowerSwitch){
   chgVoltage = 0.0; 
}

