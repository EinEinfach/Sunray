#include <Arduino.h>
#include "pid.h"

PID::PID()
{
    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    outputMin = 0;
    outputMax = 0;
    sampleTime = 0;
    lastComputeTime = 0;
    output = 0;
    lastError = 0;
    integral;
}

void PID::setup(float kP, float kI, float kD, int outputMin, int outputMax, int spampleTime)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->outputMin = outputMin;
    this->outputMax = outputMax;
    this->sampleTime = sampleTime;
}

void PID::reset()
{
    integral = 0;
    lastError = 0;
}

int PID::coumpute(int currentSetPoint, int currentValue)
{
    int now = millis();
    if ((now - lastComputeTime) < sampleTime)
    {
        return output;
    }
    else
    {
        int error = currentSetPoint - currentValue;
        integral += error * sampleTime;
        int derivate = (error - lastError) / sampleTime;
        output = kP * error + kI * integral + kD * derivate;
        if (output > outputMax) output = outputMax;
        else if (output < outputMin) output = outputMin;
        return output;
    }
}