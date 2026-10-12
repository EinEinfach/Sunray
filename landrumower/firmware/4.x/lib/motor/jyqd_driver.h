// JYQD brushless driver on three GPIO pins: speed (PWM), direction, brake (PWM).

#ifndef JYQD_DRIVER_H
#define JYQD_DRIVER_H

#ifdef ARDUINO

#include "motor.h"

struct JyqdPins
{
    uint8_t pwm;
    uint8_t direction;
    uint8_t brake;
    bool directionHighIsForward;
    bool brakeHighActive;
};

class JyqdDriver : public MotorDriver
{
public:
    // The speed and the brake pin must be on different PWM slices to use different frequencies.
    JyqdDriver(const JyqdPins &pins, uint32_t pwmHz, uint32_t brakeHz);

    // sets the pins to drive off, brake off
    void begin();
    void apply(const MotorOutput &output) override;

private:
    JyqdPins pins;
    uint32_t pwmHz;
    uint32_t brakeHz;
    uint32_t pwmTop = 0;
    uint32_t brakeTop = 0;
};

#endif // ARDUINO

#endif // JYQD_DRIVER_H
