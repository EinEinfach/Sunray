// Encoder on a GPIO pin, counted by interrupt on rising edges.

#ifndef PIN_ENCODER_H
#define PIN_ENCODER_H

#ifdef ARDUINO

#include "encoder.h"

class PinEncoder : public EncoderSource
{
public:
    // edges closer together than minEdgeIntervalMicros are treated as spikes
    PinEncoder(uint8_t pin, uint32_t minEdgeIntervalMicros);

    // must be called on the core that runs the control loop
    void begin();
    EncoderSample sample() override;

private:
    static void onEdge(void *self);

    uint8_t pin;
    uint32_t minEdgeIntervalMicros;
    volatile uint32_t ticks = 0;
    volatile uint32_t lastEdgeMicros = 0;
};

#endif // ARDUINO

#endif // PIN_ENCODER_H
