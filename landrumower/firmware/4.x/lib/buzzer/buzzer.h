// Buzzer patterns. Each change of the mower state is signalled once.
//
// BuzzerPlayer only decides whether the buzzer sounds. The pin is switched by the caller.

#ifndef BUZZER_H
#define BUZZER_H

#include <mower_state.h>
#include <stdint.h>

struct BuzzerPattern
{
    const int16_t *steps; // milliseconds, positive: sound, negative: silence
    uint8_t count;
};

namespace buzzer
{
extern const BuzzerPattern SILENCE;
extern const BuzzerPattern INFO;
extern const BuzzerPattern WARNING;
extern const BuzzerPattern IMU_CALIBRATION;
extern const BuzzerPattern SHUTDOWN;

const BuzzerPattern &patternFor(MowerState state);
} // namespace buzzer

class BuzzerPlayer
{
public:
    void play(const BuzzerPattern &pattern, uint32_t nowMillis);

    // plays the pattern of a new state. After Shutdown no other state is signalled.
    void setState(MowerState state, uint32_t nowMillis);

    // returns true while the buzzer has to sound
    bool update(uint32_t nowMillis);

private:
    const BuzzerPattern *pattern = &buzzer::SILENCE;
    uint8_t step = 0;
    uint32_t stepStartMillis = 0;
    MowerState state = MowerState::Unknown;
    bool hasState = false;
};

#endif // BUZZER_H
