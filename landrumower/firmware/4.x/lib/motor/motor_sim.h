// Simulated motor with driver and encoder, for the bench without mower and for host tests.
//
// A simple model, not a model of the real motor: the speed follows the drive duty with a
// time constant, coasts down slowly without drive and faster with the brake applied.

#ifndef MOTOR_SIM_H
#define MOTOR_SIM_H

#include "motor.h"

struct MotorSimConfig
{
    float maxTicksPerSecond = 350.0f; // speed at full drive
    float driveTimeConstant = 0.15f;  // seconds
    float coastTimeConstant = 0.6f;   // seconds, drive off
    float brakeTimeConstant = 0.05f;  // seconds, at full brake
};

class MotorSim : public MotorDriver, public EncoderSource
{
public:
    explicit MotorSim(const MotorSimConfig &config);

    void apply(const MotorOutput &output) override;
    EncoderSample sample() override;

    // advances the model to nowMicros
    void step(uint32_t nowMicros);

    // signed impulses per second, negative when turning backwards
    float velocity() const { return ticksPerSecond; }

private:
    MotorSimConfig config;
    MotorOutput output;
    EncoderSample encoder;
    float ticksPerSecond = 0.0f;
    float position = 0.0f; // fraction of the way to the next impulse
    uint32_t lastStepMicros = 0;
    bool started = false;
};

#endif // MOTOR_SIM_H
