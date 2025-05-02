#include <Arduino.h>
#include "ansensor.h"

AnSensor::AnSensor(uint8_t pin, bool inverse, uint16_t threshold) {
    triggered = false;
}