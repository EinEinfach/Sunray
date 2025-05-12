#include <Arduino.h>
#include <BindArg.h>
#include "config.h"
#include "motor.h"

Motor::Motor(uint8_t pinImp, uint8_t pinPwm, uint8_t pinDir, uint8_t pinBrake, uint8_t inaAddress, float inaShunt)
    : ina(INALEFTADRESS)
{
    odomTicks = 0;
    ticksTimeout = 0;
    messageTimeout = 0;
    overload = false;
    electricalCurrent = 0.0;
    this->pinImp = pinImp;
    this->pinPwm = pinPwm;
    this->pinDir = pinDir;
    this->pinBrake = pinBrake;
    this->inaShunt = inaShunt;
    nextRunTime = 0;
    nextCurrRunTime = 0;
}

void Motor::setup()
{
    pinMode(pinImp, INPUT);
    pinMode(pinPwm, OUTPUT);
    pinMode(pinDir, OUTPUT);
    pinMode(pinBrake, OUTPUT);
    interruptGate = bindArgGateThisAllocate<Motor>(&Motor::odometryIsr, this);
    attachInterrupt(digitalPinToInterrupt(pinImp), interruptGate, RISING);
    if (!HIL)
        connectSensor();
    else
        sensorConnected = true;
}

void Motor::run()
{
    int now = millis();
    if ((nextCurrRunTime - now) < 0)
    {
        if (!sensorConnected)
            connectSensor();
        else
            electricalCurrent = ina.getCurrent();
        nextCurrRunTime = now + 100;
    }
}

void Motor::connectSensor()
{
    sensorConnected = ina.begin();
    if (!sensorConnected)
    {
        USB.println("Connection to motor ina sensor failed");
        return;
    }
    ina.configure(inaShunt);
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