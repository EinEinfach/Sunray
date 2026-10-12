#include "mower_state.h"

MowerState mowerStateFromSunray(int sunrayState)
{
    switch (sunrayState)
    {
    case 0:
        return MowerState::Idle;
    case 1:
        return MowerState::Mow;
    case 2:
        return MowerState::Charge;
    case 3:
        return MowerState::Error;
    case 4:
        return MowerState::Dock;
    case 5:
        return MowerState::EscapeForward;
    case 6:
        return MowerState::EscapeReverse;
    case 7:
        return MowerState::GpsRecovery;
    case 8:
        return MowerState::GpsWaitFix;
    case 9:
        return MowerState::GpsWaitFloat;
    case 10:
        return MowerState::ImuCalibration;
    case 11:
        return MowerState::KidnapWait;
    default:
        return MowerState::Unknown;
    }
}

const char *mowerStateText(MowerState state)
{
    switch (state)
    {
    case MowerState::Boot:
        return "boot";
    case MowerState::Idle:
        return "idle";
    case MowerState::Mow:
        return "mow";
    case MowerState::Charge:
        return "charge";
    case MowerState::Error:
        return "error";
    case MowerState::Dock:
        return "dock";
    case MowerState::EscapeForward:
        return "escape forward";
    case MowerState::EscapeReverse:
        return "escape reverse";
    case MowerState::GpsRecovery:
        return "gps recovery";
    case MowerState::GpsWaitFix:
        return "gps wait fix";
    case MowerState::GpsWaitFloat:
        return "gps wait float";
    case MowerState::ImuCalibration:
        return "imu calibration";
    case MowerState::KidnapWait:
        return "kidnap wait";
    case MowerState::Shutdown:
        return "shutdown";
    case MowerState::Unknown:
        break;
    }
    return "unknown";
}
