#include <Arduino.h>
#include <BindArg.h>
#include "config.h"
#include "motor.h"

Motor::Motor(String type, uint8_t pinImp, uint8_t pinPwm, uint8_t pinDir, bool positiveDirHighActive, uint8_t pinBrake, bool brakeHighActive, uint8_t inaAddress, float inaShunt)
    : ina(INALEFTADRESS), pid()
{
    this->type = type;
    this->pinImp = pinImp;
    this->pinPwm = pinPwm;
    this->pinDir = pinDir;
    this->positiveDirHighActive = positiveDirHighActive;
    this->pinBrake = pinBrake;
    this->brakeHighActive = brakeHighActive;
    this->inaShunt = inaShunt;
    odomTicks = 0;
    lastMeasuredOdomTicks = 0;
    ticksTimeout = 0;
    messageTimeout = 0;
    overload = false;
    overloadTimeout = 0;
    electricalCurrent = 0.0;
    currentPwm = 0;
    currentTrqPwm = 0;
    currentBrakePwm;
    currentRpm = 0;
    currentSetPoint = 0;
    positiveDirection = true;
    nextRunTime = 0;
    nextCurrRunTime = 0;
    lastCalcSpeedTime = 0;
    wheelCircumference = int(3.1415 * WHEELDIAMETER * 1000);
}

void Motor::setup()
{
    pinMode(pinImp, INPUT);
    pinMode(pinPwm, OUTPUT);
    pinMode(pinDir, OUTPUT);
    pinMode(pinBrake, OUTPUT);
    //analogWriteFreq(FREQ);
    analogWriteResolution(16);
    interruptGate = bindArgGateThisAllocate<Motor>(&Motor::odometryIsr, this);
    attachInterrupt(digitalPinToInterrupt(pinImp), interruptGate, RISING);
    if (!HIL)
        connectSensor();
    else
        sensorConnected = true;
    if (type == "gear")
        currentThreshold = OVERLOADCURRENT_GEAR;
    else
        currentThreshold = OVERLOADCURRENT_MOW;
    pid.setup(2, 0.0, 0.0, 0, 65535, 10);
}

void Motor::run()
{
    checkTimeout();
    readCurrent();
    checkOverload();
    calcSpeed();

    if (PICOMOTORCONTROL)
    {
        currentTrqPwm = currentTrqPwm + pid.coumpute(currentSetPoint, currentSpeed);
    }
    setDriverPins();
}

void Motor::odometryIsr()
{
    if (digitalRead(pinImp) == LOW)
        return;
    if (millis() < ticksTimeout)
        return; // eliminate spikes
    ticksTimeout = millis() + 1;
    odomTicks++;
}

void Motor::connectSensor()
{
    sensorConnected = ina.begin();
    if (!sensorConnected)
    {
        USB.println("Connection to motor ina sensor failed");
        return;
    }
    ina.configure(inaShunt);
}

void Motor::checkTimeout()
{
    int now = millis();
    if ((messageTimeout - now) < 0)
        stop();
}

void Motor::readCurrent()
{
    int now = millis();
    if ((nextCurrRunTime - now) < 0)
    {
        if (HIL)
        {
            electricalCurrent = 0.5;
        }
        else if (!sensorConnected)
        {
            connectSensor();
        }
        else
        {
            electricalCurrent = ina.getCurrent();
        }
        nextCurrRunTime = now + 100;
    }
}

void Motor::checkOverload()
{
    int now = millis();
    if (electricalCurrent > currentThreshold)
    {
        overload = true;
        stop();
        overloadTimeout = now + 2000;
    }
    else
    {
        overload = false;
    }
}

void Motor::setDriverPins()
{
    currentTrqPwm = std::max(0, std::min(currentTrqPwm, 65535));
    currentBrakePwm = std::max(0, std::min(currentBrakePwm, 65535));
    bool dirState = positiveDirection ? positiveDirHighActive : !positiveDirHighActive;
    digitalWrite(pinDir, dirState);
    analogWrite(pinPwm, currentTrqPwm);
    // analogWrite(pinBrake, currentBrakePwm);
    digitalWrite(pinBrake, LOW);
}

void Motor::setSpeed(int setPoint)
{
    int now = millis();
    messageTimeout = now + 3000;
    if (overload && (overloadTimeout - now) > 0)
    {
        return;
    }

    // at first check if direction change
    if ((currentSetPoint * setPoint) < 0)
    {
        currentSetPoint = setPoint;
        positiveDirection = !positiveDirection;
        stop();
        pid.reset();
        return;
    }

    // check direction of speed
    positiveDirection = false ? setPoint < 0 : true;
    currentSetPoint = setPoint;
    if (PICOMOTORCONTROL)
    {
        currentTrqPwm = pid.coumpute(abs(currentSetPoint), currentSpeed);
    }
    else
    {
        currentTrqPwm = (abs(currentSetPoint) * 65535) / 255;
    }
}

void Motor::calcSpeed()
{
    int now = millis();
    int timeDelta = now - lastCalcSpeedTime;
    if (timeDelta > 10)
    {
        int currentTicks = odomTicks - lastMeasuredOdomTicks;
        lastMeasuredOdomTicks = odomTicks;
        currentRpm = (60000 * currentTicks) / (TICKSPERREVOLUTION * timeDelta);

        currentSpeed = (1000 * wheelCircumference * currentTicks) / (TICKSPERREVOLUTION * timeDelta);

        lastCalcSpeedTime = now;
    }
}

void Motor::stop()
{
    currentSetPoint = 0;
    currentPwm = 0;
    currentTrqPwm = 0;
    currentBrakePwm = 0 ? brakeHighActive : 65535;
    // motorControl.reset();
}