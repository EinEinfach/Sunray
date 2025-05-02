#include <Arduino.h>

class DigSensor
{
public:
    DigSensor(uint8_t pin, bool inverse);
    bool triggered;
};