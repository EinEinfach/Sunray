#include "protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace protocol
{

namespace
{

constexpr int MAX_FIELDS = 8;

bool parseInt(const char *text, int &value)
{
    if (*text == '\0')
        return false;
    char *end = nullptr;
    const long parsed = strtol(text, &end, 10);
    if (*end != '\0')
        return false;
    value = static_cast<int>(parsed);
    return true;
}

bool parseFloat(const char *text, float &value)
{
    if (*text == '\0')
        return false;
    char *end = nullptr;
    const float parsed = strtof(text, &end);
    if (*end != '\0')
        return false;
    value = parsed;
    return true;
}

// Removes a trailing ",0x.." from line. Returns false if the CRC is wrong, or missing although required.
bool stripCrc(char *line, bool requireCrc)
{
    char *comma = strrchr(line, ',');
    const bool present = (comma != nullptr) && (comma[1] == '0') && (comma[2] == 'x' || comma[2] == 'X');
    if (!present)
        return !requireCrc;
    char *end = nullptr;
    const long received = strtol(comma + 1, &end, 16);
    if ((end == comma + 3) || (*end != '\0'))
        return false;
    if (received != checksum(line, static_cast<size_t>(comma - line)))
        return false;
    *comma = '\0';
    return true;
}

// Splits line at the commas in place and returns the number of fields.
int split(char *line, char *fields[], int maxFields)
{
    int count = 0;
    fields[count++] = line;
    for (char *p = line; *p != '\0'; p++)
    {
        if (*p != ',')
            continue;
        *p = '\0';
        if (count == maxFields)
            return maxFields + 1;
        fields[count++] = p + 1;
    }
    return count;
}

bool parseMotor(char *fields[], int count, Command &command)
{
    if ((count != 4) && (count != 6))
        return false;
    if (!parseInt(fields[1], command.rightPwm) || !parseInt(fields[2], command.leftPwm) || !parseInt(fields[3], command.mowPwm))
        return false;
    if (count == 6)
    {
        if (!parseInt(fields[4], command.rightSpeed) || !parseInt(fields[5], command.leftSpeed))
            return false;
        command.hasSpeed = true;
    }
    return true;
}

bool parseSummary(char *fields[], int count, Command &command)
{
    if ((count != 1) && (count != 2) && (count != 5))
        return false;
    // Sunray sends "AT+S," with an empty state between two state updates
    if ((count >= 2) && (fields[1][0] != '\0'))
    {
        if (!parseInt(fields[1], command.state))
            return false;
        command.hasState = true;
    }
    if (count == 5)
    {
        if (!parseFloat(fields[2], command.kp) || !parseFloat(fields[3], command.ki) || !parseFloat(fields[4], command.kd))
            return false;
        command.hasGains = true;
    }
    return true;
}

} // namespace

uint8_t checksum(const char *data, size_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++)
        sum += static_cast<uint8_t>(data[i]);
    return sum;
}

bool parseCommand(const char *line, bool requireCrc, Command &command)
{
    command = Command();
    char work[MAX_LINE];
    const size_t length = strlen(line);
    if (length >= sizeof(work))
        return false;
    memcpy(work, line, length + 1);

    if (!stripCrc(work, requireCrc))
        return false;

    char *fields[MAX_FIELDS];
    const int count = split(work, fields, MAX_FIELDS);
    if (count > MAX_FIELDS)
        return false;

    Command parsed;
    const char *name = fields[0];
    if (strcmp(name, "AT+M") == 0)
    {
        if (!parseMotor(fields, count, parsed))
            return false;
        parsed.type = CommandType::Motor;
    }
    else if (strcmp(name, "AT+S") == 0)
    {
        if (!parseSummary(fields, count, parsed))
            return false;
        parsed.type = CommandType::Summary;
    }
    else if (count != 1)
        return false;
    else if (strcmp(name, "AT+V") == 0)
        parsed.type = CommandType::Version;
    else if (strcmp(name, "AT+R") == 0)
        parsed.type = CommandType::ResetFaults;
    else if (strcmp(name, "AT+E") == 0)
        parsed.type = CommandType::MotorTest;
    else if (strcmp(name, "AT+Y") == 0)
        parsed.type = CommandType::TriggerWatchdog;
    else if (strcmp(name, "AT+Y3") == 0)
        parsed.type = CommandType::Shutdown;
    else
        return false;

    command = parsed;
    return true;
}

size_t formatFrame(char *out, size_t size, const char *payload)
{
    const size_t length = strlen(payload);
    const int written = snprintf(out, size, "%s,0x%02x\r\n", payload, checksum(payload, length));
    if ((written < 0) || (static_cast<size_t>(written) >= size))
    {
        if (size > 0)
            out[0] = '\0';
        return 0;
    }
    return static_cast<size_t>(written);
}

size_t formatVersion(char *out, size_t size, const char *name, const char *version)
{
    char payload[MAX_LINE];
    const int written = snprintf(payload, sizeof(payload), "V,%s,%s", name, version);
    if ((written < 0) || (static_cast<size_t>(written) >= sizeof(payload)))
        return 0;
    return formatFrame(out, size, payload);
}

size_t formatMotor(char *out, size_t size, const MotorReport &report)
{
    char payload[MAX_LINE];
    const int written = snprintf(payload, sizeof(payload), "M,%ld,%ld,%ld,%.2f,%d,%d,%d",
                                 report.rightTicks, report.leftTicks, report.mowTicks,
                                 static_cast<double>(report.chargeVoltage),
                                 report.bumper, report.lift, report.stopButton);
    if ((written < 0) || (static_cast<size_t>(written) >= sizeof(payload)))
        return 0;
    return formatFrame(out, size, payload);
}

size_t formatSummary(char *out, size_t size, const SummaryReport &report)
{
    char payload[MAX_LINE];
    const int written = snprintf(payload, sizeof(payload), "S,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f",
                                 static_cast<double>(report.batteryVoltage),
                                 static_cast<double>(report.chargeVoltage),
                                 static_cast<double>(report.chargeCurrent),
                                 report.lift, report.bumper, report.rain, report.motorFault,
                                 static_cast<double>(report.mowCurrent),
                                 static_cast<double>(report.leftCurrent),
                                 static_cast<double>(report.rightCurrent),
                                 static_cast<double>(report.batteryTemperature));
    if ((written < 0) || (static_cast<size_t>(written) >= sizeof(payload)))
        return 0;
    return formatFrame(out, size, payload);
}

bool LineReader::feed(char ch)
{
    if (complete)
    {
        complete = false;
        length = 0;
    }
    if ((ch == '\r') || (ch == '\n'))
    {
        const bool usable = (length > 0) && !overflow;
        buffer[length] = '\0';
        overflow = false;
        if (!usable)
        {
            length = 0;
            return false;
        }
        complete = true;
        return true;
    }
    if (length < MAX_LINE - 1)
        buffer[length++] = ch;
    else
        overflow = true;
    return false;
}

} // namespace protocol
