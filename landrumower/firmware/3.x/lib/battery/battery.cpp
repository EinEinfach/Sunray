#include <Arduino.h>
#include "config.h"
#include "battery.h"

Battery::Battery(uint8_t pinPowerSwitch) : ina(INABATADRESS, &Wire)
{
    this->pinPowerSwitch = pinPowerSwitch;
    nextRunTime = 0;
    nextRunTimeHighFreq = 0;
    voltage = 24.0;
    chgVoltage = 0.0;
    chgCurrent = 0.0;
    temperature = 0.0;
}

void Battery::setup()
{
    pinMode(pinPowerSwitch, OUTPUT);
    digitalWrite(pinPowerSwitch, HIGH);
    if (!HIL)
    {
        connectSensor();
    }
}

void Battery::run()
{
    int now = millis();
    if ((nextRunTime - now) < 0)
    {
        if (!HIL)
            if (sensorConnected)
                readVoltage();
            else
                connectSensor();
        else
            voltage = 24.0;
        nextRunTime = now + 500;
    }
    if ((nextRunTimeHighFreq - now) < 0)
    {
        if (!HIL)
            if (sensorConnected)
                readCurrent();
            else
                chgCurrent = 0.5;
        nextRunTimeHighFreq = now + 20;
    }
}

void Battery::connectSensor()
{
    sensorConnected = ina.begin();
    if (!sensorConnected)
    {
        USB.println("Connection to battery ina sensor failed");
        return;
    }
    ina.configure(INABATSHUNT);
}

void Battery::readVoltage()
{
    float w = 0.99;
    float voltageRaw = 0.0;
    voltageRaw = ina.getBusVoltage() + ina.getShuntVoltage();
    voltage = w * voltage + (1-w) * voltageRaw;
}

void Battery::readCurrent()
{
    float w = 0.99;
    float currentRaw = 0.0;
    currentRaw = ina.getCurrent();
    chgCurrent = w * chgCurrent + (1-w) * currentRaw;
}
