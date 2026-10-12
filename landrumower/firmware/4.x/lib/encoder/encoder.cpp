#include "encoder.h"

SpeedEstimator::SpeedEstimator(uint32_t timeoutMicros)
    : timeoutMicros(timeoutMicros)
{
}

float SpeedEstimator::update(const EncoderSample &sample, uint32_t nowMicros)
{
    if (!started)
    {
        started = true;
        last = sample;
        return speed;
    }

    // unsigned differences stay correct when the counters wrap
    const uint32_t ticks = sample.ticks - last.ticks;
    if (ticks > 0)
    {
        const uint32_t elapsed = sample.lastEdgeMicros - last.lastEdgeMicros;
        // the first impulse after standstill has no predecessor to measure against
        if (hasEdgeTime && (elapsed > 0))
            speed = static_cast<float>(ticks) * 1e6f / static_cast<float>(elapsed);
        last = sample;
        hasEdgeTime = true;
        return speed;
    }

    const uint32_t idle = nowMicros - last.lastEdgeMicros;
    if (!hasEdgeTime || (idle >= timeoutMicros))
    {
        speed = 0.0f;
        hasEdgeTime = false;
        return speed;
    }
    // no impulse yet: the motor cannot be faster than one impulse per idle time
    const float bound = 1e6f / static_cast<float>(idle);
    if (bound < speed)
        speed = bound;
    return speed;
}
