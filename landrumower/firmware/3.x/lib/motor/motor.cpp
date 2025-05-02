#include <Arduino.h>
#include <BindArg.h>
#include "config.h"
#include "motor.h"

Motor::Motor(uint8_t pinImp, uint8_t pinPwm, uint8_t pinDir, uint8_t pinBrake)
{
    odomTicks = 0;
    ticksTimeout = 0;
    overload = false;
    electricalCurrent = 0.0;
    this->pinImp = pinImp;
    this->pinPwm = pinPwm;
    this->pinDir = pinDir;
    this->pinBrake = pinBrake;
}

void Motor::setup()
{
    pinMode(pinImp, INPUT);
    pinMode(pinPwm, OUTPUT);
    pinMode(pinDir, OUTPUT);
    pinMode(pinBrake, OUTPUT);
    interruptGate = bindArgGateThisAllocate<Motor>(&Motor::odometryIsr, this);
    attachInterrupt(digitalPinToInterrupt(pinImp), interruptGate, RISING);
}

void Motor::odometryIsr()
{
    if (digitalRead(pinImp) == LOW)
        return;
    if (millis() < ticksTimeout)
        return; // eliminate spikes
    ticksTimeout = millis() + 1;
    odomTicks++;
}