#include <Arduino.h>
#include "config.h"
#include "battery.h"

Battery::Battery(uint8_t pinPowerSwitch) : ina(INABATADRESS, &Wire)
{
    this->pinPowerSwitch = pinPowerSwitch;
    nextRunTime = 0;
    nextRunTimeHighFreq = 0;
    chgConnected = false;
    requestShutdown = false;
    voltage = 24.0;
    chgVoltage = 0.0;
    chgCurrent = 0.0;
    temperature = 0.0;
    shutdownRequestTime = 0;
}

void Battery::setup()
{
    pinMode(pinPowerSwitch, OUTPUT);
    digitalWrite(pinPowerSwitch, HIGH);
    if (!HIL)
        connectSensor();
    else
        sensorConnected = true;
}

void Battery::run()
{
    int now = millis();
    keepPowerOn();
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
            {
                readCurrent();
                checkCharger();
            }
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
    float w = 0.9;
    float voltageRaw = 0.0;
    voltageRaw = ina.getBusVoltage() + ina.getShuntVoltage();
    voltage = w * voltage + (1 - w) * voltageRaw;
}

void Battery::readCurrent()
{
    float w = 0.9;
    float currentRaw = 0.0;
    currentRaw = ina.getCurrent();
    chgCurrent = w * chgCurrent + (1 - w) * currentRaw;
}

void Battery::checkCharger()
{
    if (chgCurrent < CHGCONNECTEDCURRENT)
    {
        chgConnected = true;
        chgVoltage = voltage;
    }
    else
    {
        chgConnected = false;
        chgVoltage = 0.0;
    }
}

void Battery::keepPowerOn()
{
    // main unit request
    if (requestShutdown && (shutdownRequestTime - millis() + 30000) < 0)
    {
        digitalWrite(pinPowerSwitch, LOW);
    }
    // emergency shutdown
    if (voltage < CRITICALVOLTAGE) {
        digitalWrite(pinPowerSwitch, LOW);
    }
}
