#include "buzzer.h"

namespace buzzer
{

namespace
{
const int16_t INFO_STEPS[] = {1000};
const int16_t WARNING_STEPS[] = {500, -1000, 500, -1000, 500};
const int16_t IMU_CALIBRATION_STEPS[] = {
    100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000,
    100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000, 100};
const int16_t SHUTDOWN_STEPS[] = {
    100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000,
    100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000};

template <uint8_t N>
constexpr BuzzerPattern patternOf(const int16_t (&steps)[N])
{
    return {steps, N};
}
} // namespace

const BuzzerPattern SILENCE = {nullptr, 0};
const BuzzerPattern INFO = patternOf(INFO_STEPS);
const BuzzerPattern WARNING = patternOf(WARNING_STEPS);
const BuzzerPattern IMU_CALIBRATION = patternOf(IMU_CALIBRATION_STEPS);
const BuzzerPattern SHUTDOWN = patternOf(SHUTDOWN_STEPS);

const BuzzerPattern &patternFor(MowerState state)
{
    switch (state)
    {
    case MowerState::Boot:
    case MowerState::Idle:
    case MowerState::Charge:
    case MowerState::Dock:
    case MowerState::GpsRecovery:
    case MowerState::GpsWaitFix:
    case MowerState::GpsWaitFloat:
        return INFO;
    case MowerState::Mow:
    case MowerState::Error:
    case MowerState::EscapeForward:
    case MowerState::EscapeReverse:
        return WARNING;
    case MowerState::ImuCalibration:
        return IMU_CALIBRATION;
    case MowerState::Shutdown:
        return SHUTDOWN;
    case MowerState::KidnapWait:
    case MowerState::Unknown:
        break;
    }
    return SILENCE;
}

} // namespace buzzer

void BuzzerPlayer::play(const BuzzerPattern &newPattern, uint32_t nowMillis)
{
    pattern = &newPattern;
    step = 0;
    stepStartMillis = nowMillis;
}

void BuzzerPlayer::setState(MowerState newState, uint32_t nowMillis)
{
    if (hasState && (newState == state))
        return;
    if (hasState && (state == MowerState::Shutdown))
        return;
    hasState = true;
    state = newState;
    play(buzzer::patternFor(newState), nowMillis);
}

bool BuzzerPlayer::update(uint32_t nowMillis)
{
    while (step < pattern->count)
    {
        const int16_t duration = pattern->steps[step];
        const uint32_t length = static_cast<uint32_t>(duration < 0 ? -duration : duration);
        if (nowMillis - stepStartMillis < length)
            return duration > 0;
        stepStartMillis += length;
        step++;
    }
    return false;
}
