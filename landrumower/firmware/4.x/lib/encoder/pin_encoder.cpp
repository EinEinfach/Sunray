#ifdef ARDUINO

#include "pin_encoder.h"

#include <Arduino.h>

PinEncoder::PinEncoder(uint8_t pin, uint32_t minEdgeIntervalMicros)
    : pin(pin), minEdgeIntervalMicros(minEdgeIntervalMicros)
{
}

void PinEncoder::begin()
{
    pinMode(pin, INPUT);
    attachInterruptParam(digitalPinToInterrupt(pin), onEdge, RISING, this);
}

EncoderSample PinEncoder::sample()
{
    EncoderSample result;
    noInterrupts();
    result.ticks = ticks;
    result.lastEdgeMicros = lastEdgeMicros;
    interrupts();
    return result;
}

void PinEncoder::onEdge(void *self)
{
    PinEncoder *encoder = static_cast<PinEncoder *>(self);
    const uint32_t now = time_us_32();
    if ((encoder->ticks > 0) && (now - encoder->lastEdgeMicros < encoder->minEdgeIntervalMicros))
        return;
    encoder->lastEdgeMicros = now;
    encoder->ticks = encoder->ticks + 1;
}

#endif // ARDUINO
