// AT protocol between Sunray (SerialRobotDriver) and the Pico.
//
// Frame:  <payload>,0x<crc>\r\n   crc = sum of all payload bytes modulo 256
//
// No Arduino dependency and no heap, so it builds for the host tests as well.

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

namespace protocol
{

// longest line that is accepted or produced, including CRC and terminator
constexpr size_t MAX_LINE = 128;

enum class CommandType : uint8_t
{
    None,
    Version,         // AT+V
    Motor,           // AT+M,rightPwm,leftPwm,mowPwm[,rightSpeed,leftSpeed]
    Summary,         // AT+S[,state[,kp,ki,kd]]
    ResetFaults,     // AT+R
    MotorTest,       // AT+E
    TriggerWatchdog, // AT+Y
    Shutdown         // AT+Y3
};

struct Command
{
    CommandType type = CommandType::None;

    // AT+M, pwm -255..255, speed in mm/s
    int rightPwm = 0;
    int leftPwm = 0;
    int mowPwm = 0;
    bool hasSpeed = false;
    int rightSpeed = 0;
    int leftSpeed = 0;

    // AT+S, state is the Sunray operation (see robot.cpp), gains are sent once after AT+CM
    bool hasState = false;
    int state = 0;
    bool hasGains = false;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

struct MotorReport
{
    long rightTicks = 0;
    long leftTicks = 0;
    long mowTicks = 0;
    float chargeVoltage = 0.0f;
    bool bumper = false;
    bool lift = false;
    bool stopButton = false;
};

struct SummaryReport
{
    float batteryVoltage = 0.0f;
    float chargeVoltage = 0.0f;
    float chargeCurrent = 0.0f;
    bool lift = false;
    bool bumper = false;
    bool rain = false;
    bool motorFault = false;
    float mowCurrent = 0.0f;
    float leftCurrent = 0.0f;
    float rightCurrent = 0.0f;
    float batteryTemperature = 0.0f;
};

uint8_t checksum(const char *data, size_t length);

// Parses one line without terminator. A CRC that is present must match. A missing CRC is
// only accepted if requireCrc is false (USB console).
bool parseCommand(const char *line, bool requireCrc, Command &command);

// Writers append CRC and "\r\n". They return the string length, or 0 if it does not fit.
size_t formatFrame(char *out, size_t size, const char *payload);
size_t formatVersion(char *out, size_t size, const char *name, const char *version);
size_t formatMotor(char *out, size_t size, const MotorReport &report);
size_t formatSummary(char *out, size_t size, const SummaryReport &report);

// Collects bytes of one serial port into lines.
class LineReader
{
public:
    // returns true if ch completed a line, which is then valid until the next call
    bool feed(char ch);
    const char *line() const { return buffer; }

private:
    char buffer[MAX_LINE] = {0};
    size_t length = 0;
    bool complete = false;
    bool overflow = false;
};

} // namespace protocol

#endif // PROTOCOL_H
