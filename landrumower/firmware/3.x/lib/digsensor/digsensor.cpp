#include <Arduino.h>
#include "digsensor.h"

DigSensor::DigSensor(uint8_t pin, bool inverse)
{
    this->pin = pin;
    this->inverse = inverse;
    triggered = false;
}

void DigSensor::setup()
{
    if (inverse)
        pinMode(pin, INPUT_PULLUP);
    else
        pinMode(pin, INPUT_PULLDOWN);
}

void DigSensor::run()
{
    int now = millis();
    if ((nextRunTime - now) < 0)
    {
        if (inverse)
            triggered = digitalRead(pin) != inverse;
        else
            triggered = digitalRead(pin) == inverse;
        nextRunTime = now + 500;
    }
}
