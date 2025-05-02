#include <Arduino.h>
#include "digsensor.h"

DigSensor::DigSensor(uint8_t pin, bool inverse){
    triggered = false;
}

