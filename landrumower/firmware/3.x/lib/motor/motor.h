#include <Arduino.h>
#include <BindArg.h>
#include <INA226.h>
#include "config.h"
#include "pid.h"

class Motor
{
public:
    Motor(String type, uint8_t pinImp, uint8_t pinPwm, uint8_t pinDir, bool positiveDirHighActive, uint8_t pinBrake, bool brakeHighActive, uint8_t inaAddress, float inaShunt);
    String type;
    uint64_t odomTicks;
    float electricalCurrent;
    bool overload;
    int currentPwm;
    int currentTrqPwm;
    int currentBrakePwm;
    int currentRpm;
    int currentSpeed;
    int currentSetPoint;
    bool positiveDirection;
    void setup();
    void run();
    void setSpeed(int);
private:
    int nextRunTime;
    int nextCurrRunTime;
    bool sensorConnected;
    float currentThreshold;
    int overloadTimeout;
    float inaShunt;
    uint8_t pinImp;
    uint8_t pinPwm;
    uint8_t pinDir;
    bool positiveDirHighActive;
    uint8_t pinBrake;
    bool brakeHighActive;
    int messageTimeout;
    int ticksTimeout;
    int lastCalcSpeedTime;
    int lastMeasuredOdomTicks;
    int wheelCircumference;
    INA226 ina;
    PID pid;
    void odometryIsr();
    void connectSensor();
    void checkTimeout();
    void readCurrent();
    void checkOverload();
    void setDriverPins();
    void stop();
    void calcSpeed();
protected:
    bindArgVoidFunc_t interruptGate = nullptr;
};