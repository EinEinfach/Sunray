#include "motor_sim.h"

#include <math.h>

namespace
{
// longer gaps are cut off, the model is integrated with a simple Euler step
constexpr float MAX_STEP = 0.02f;
} // namespace

MotorSim::MotorSim(const MotorSimConfig &config)
    : config(config)
{
}

void MotorSim::apply(const MotorOutput &newOutput)
{
    output = newOutput;
}

EncoderSample MotorSim::sample()
{
    return encoder;
}

void MotorSim::step(uint32_t nowMicros)
{
    if (!started)
    {
        started = true;
        lastStepMicros = nowMicros;
        return;
    }
    const uint32_t stepStart = lastStepMicros;
    float dt = static_cast<float>(nowMicros - lastStepMicros) * 1e-6f;
    lastStepMicros = nowMicros;
    if (dt <= 0.0f)
        return;
    if (dt > MAX_STEP)
        dt = MAX_STEP;

    float acceleration;
    if (output.drive > 0.0f)
    {
        const float goal = (output.forward ? 1.0f : -1.0f) * output.drive * config.maxTicksPerSecond;
        acceleration = (goal - ticksPerSecond) / config.driveTimeConstant;
    }
    else
    {
        acceleration = -ticksPerSecond / config.coastTimeConstant;
    }
    acceleration -= ticksPerSecond * output.brake / config.brakeTimeConstant;
    ticksPerSecond += acceleration * dt;

    // the encoder counts in both directions
    const float travelled = fabsf(ticksPerSecond) * dt;
    const float before = position;
    position += travelled;
    if (position >= 1.0f)
    {
        const float whole = floorf(position);
        position -= whole;
        encoder.ticks += static_cast<uint32_t>(whole);
        // the latest impulse happened where the position crossed its last whole number
        const float fraction = (whole - before) / travelled;
        encoder.lastEdgeMicros = stepStart + static_cast<uint32_t>(fraction * dt * 1e6f);
    }
}
