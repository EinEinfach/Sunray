#include <Arduino.h>
#include <BindArg.h>
#include <INA226.h>
#include "config.h"

class Motor
{
public:
    Motor(uint8_t pinImp, uint8_t pinPwm, uint8_t pinDir, uint8_t pinBrake, uint8_t inaAddress, float inaShunt);
    uint64_t odomTicks;
    bool overload;
    float electricalCurrent;
    void setup();
    void run();
private:
    int nextRunTime;
    int nextCurrRunTime;
    bool sensorConnected;
    float inaShunt;
    uint8_t pinImp;
    uint8_t pinPwm;
    uint8_t pinDir;
    uint8_t pinBrake;
    uint ticksTimeout;
    INA226 ina;
    void odometryIsr();
    void connectSensor();
protected:
    bindArgVoidFunc_t interruptGate = nullptr;
};