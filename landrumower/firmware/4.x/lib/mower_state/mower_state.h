// What the mower is doing, as shown on the LCD and signalled by the buzzer.
//
// Sunray sends its operation as a number in AT+S (see robot.cpp). Boot and Shutdown are
// states of the Pico itself.

#ifndef MOWER_STATE_H
#define MOWER_STATE_H

enum class MowerState
{
    Boot,
    Unknown,
    Idle,
    Mow,
    Charge,
    Error,
    Dock,
    EscapeForward,
    EscapeReverse,
    GpsRecovery,
    GpsWaitFix,
    GpsWaitFloat,
    ImuCalibration,
    KidnapWait,
    Shutdown
};

MowerState mowerStateFromSunray(int sunrayState);
const char *mowerStateText(MowerState state);

#endif // MOWER_STATE_H
