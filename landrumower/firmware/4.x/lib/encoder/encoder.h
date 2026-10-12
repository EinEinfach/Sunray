// Encoder input and speed measurement.
//
// The motor drivers deliver one impulse line per motor, without direction. At walking speed
// that is only about 150 impulses per second, so counting impulses per control step is far
// too coarse. The speed is derived from the time between impulses instead.

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

struct EncoderSample
{
    uint32_t ticks = 0;          // impulses since start, wraps around
    uint32_t lastEdgeMicros = 0; // time of the latest impulse
};

class EncoderSource
{
public:
    virtual ~EncoderSource() = default;
    virtual EncoderSample sample() = 0;
};

class SpeedEstimator
{
public:
    // without an impulse for timeoutMicros the motor counts as stopped
    explicit SpeedEstimator(uint32_t timeoutMicros);

    // returns impulses per second
    float update(const EncoderSample &sample, uint32_t nowMicros);

private:
    uint32_t timeoutMicros;
    EncoderSample last;
    bool started = false;
    bool hasEdgeTime = false; // last.lastEdgeMicros belongs to the current movement
    float speed = 0.0f;
};

#endif // ENCODER_H
