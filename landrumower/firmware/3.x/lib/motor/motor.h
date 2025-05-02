#include <Arduino.h>
#include <BindArg.h>
#include "config.h"

class Motor
{
public:
    Motor(uint8_t pinImp, uint8_t pinPwm, uint8_t pinDir, uint8_t pinBrake);
    uint64_t odomTicks;
    bool overload;
    float electricalCurrent;
    void setup();
private:
    uint8_t pinImp;
    uint8_t pinPwm;
    uint8_t pinDir;
    uint8_t pinBrake;
    uint ticksTimeout;
    void odometryIsr();
protected:
    bindArgVoidFunc_t interruptGate = nullptr;
};