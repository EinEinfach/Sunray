#include <Arduino.h>

class DigSensor
{
public:
    DigSensor(uint8_t pin, bool inverse);
    uint8_t pin;
    bool inverse;
    bool triggered;
    void setup();
    void run();
private:
    uint32_t nextRunTime;
};