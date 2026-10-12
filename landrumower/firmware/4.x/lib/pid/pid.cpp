#include "pid.h"

Pid::Pid(PidGains gains, float outputMin, float outputMax)
    : gains(gains), outputMin(outputMin), outputMax(outputMax)
{
}

void Pid::setGains(PidGains newGains)
{
    gains = newGains;
}

void Pid::setOutputLimits(float newOutputMin, float newOutputMax)
{
    outputMin = newOutputMin;
    outputMax = newOutputMax;
}

void Pid::reset()
{
    integral = 0.0f;
    lastMeasured = 0.0f;
    hasLastMeasured = false;
    output = 0.0f;
}

float Pid::update(float setpoint, float measured, float dt)
{
    if (dt <= 0.0f)
        return output;

    const float error = setpoint - measured;

    // derivative on the measurement, so a setpoint step causes no kick
    float derivative = 0.0f;
    if (hasLastMeasured)
        derivative = -gains.kd * (measured - lastMeasured) / dt;
    lastMeasured = measured;
    hasLastMeasured = true;

    // anti-windup: while saturated, only integrate if that leads out of saturation
    const float candidate = integral + gains.ki * error * dt;
    float result = gains.kp * error + candidate + derivative;
    if (result > outputMax)
    {
        result = outputMax;
        if (error < 0.0f)
            integral = candidate;
    }
    else if (result < outputMin)
    {
        result = outputMin;
        if (error > 0.0f)
            integral = candidate;
    }
    else
    {
        integral = candidate;
    }

    output = result;
    return output;
}
