// PID controller with output limits and anti-windup.
//
// The caller provides the time step, so the controller has no clock and no Arduino dependency.

#ifndef PID_H
#define PID_H

struct PidGains
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

class Pid
{
public:
    Pid(PidGains gains, float outputMin, float outputMax);

    void setGains(PidGains gains);
    void setOutputLimits(float outputMin, float outputMax);
    void reset();

    // dt in seconds. Returns the last output unchanged if dt is not positive.
    float update(float setpoint, float measured, float dt);

private:
    PidGains gains;
    float outputMin;
    float outputMax;
    float integral = 0.0f; // already multiplied by ki, so changing ki does not jump the output
    float lastMeasured = 0.0f;
    bool hasLastMeasured = false;
    float output = 0.0f;
};

#endif // PID_H
