#include <Arduino.h>

class AnSensor
{
public:
    AnSensor(uint8_t pin, bool inverse, uint16_t threshold);
    bool triggered;
};
