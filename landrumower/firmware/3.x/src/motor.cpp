#include <Arduino.h>
#include "config.h"
#include "motor.h"

Motor::Motor(uint8_t pinImp, uint8_t pinPwm, uint8_t pinDir, uint8_t pinBrake) {
    odomTicks = 0;
    pinImp = pinImp;
    pinPwm = pinPwm;
    pinDir = pinDir;
    pinBrake = pinBrake;
}