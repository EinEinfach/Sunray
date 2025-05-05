#include <Arduino.h>
#include "config.h"
#include "ansensor.h"

AnSensor::AnSensor(uint8_t pin, bool inverse, uint16_t threshold)
{
    this->pin = pin;
    this->inverse = inverse;
    this->threshold = threshold;
    triggered = false;
    currentValue = inverse ? 65535 : 0;
    nextRunTime = 0;
}

void AnSensor::setup()
{
    if (inverse)
        pinMode(pin, INPUT_PULLUP);
    else
        pinMode(pin, INPUT_PULLDOWN);
}

void AnSensor::run()
{
    int now = millis();
    if ((nextRunTime - now) < 0)
    {
        float w = 0.99;
        uint sensorRaw = analogRead(pin);
        currentValue = int(w*currentValue + (1-w) * sensorRaw);
        if (inverse) {
            triggered = (currentValue < threshold);
        } else {
            triggered = (currentValue > threshold);
        }
        nextRunTime = now + 500;
    }
}
