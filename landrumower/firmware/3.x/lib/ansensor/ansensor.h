#include <Arduino.h>

class AnSensor
{
public:
    AnSensor(uint8_t pin, bool inverse, uint16_t threshold);
    bool pin;
    bool inverse;
    uint16_t threshold;
    bool triggered;
    int currentValue;
    void setup();
    void run();

private:
    uint32_t nextRunTime;
};
