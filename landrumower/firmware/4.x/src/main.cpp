#include <Arduino.h>
#include <protocol.h>
#include "config.h"

// Core 0: Sunray link and, later, the motor control loop.
// Core 1: slow and blocking I/O (I2C sensors, LCD, buzzer).

namespace
{

protocol::LineReader uartReader;
protocol::LineReader usbReader;

void handleLine(Stream &port, const char *line, bool requireCrc)
{
    protocol::Command command;
    if (!protocol::parseCommand(line, requireCrc, command))
        return;

    char response[protocol::MAX_LINE];
    size_t length = 0;
    switch (command.type)
    {
    case protocol::CommandType::Version:
        length = protocol::formatVersion(response, sizeof(response), FW_NAME, FW_VERSION);
        break;
    // no motors and sensors yet: answer with empty reports to keep the link alive
    case protocol::CommandType::Motor:
        length = protocol::formatMotor(response, sizeof(response), protocol::MotorReport());
        break;
    case protocol::CommandType::Summary:
        length = protocol::formatSummary(response, sizeof(response), protocol::SummaryReport());
        break;
    default:
        break;
    }
    if (length > 0)
        port.write(response, length);
}

void poll(Stream &port, protocol::LineReader &reader, bool requireCrc)
{
    while (port.available())
    {
        if (reader.feed(static_cast<char>(port.read())))
            handleLine(port, reader.line(), requireCrc);
    }
}

} // namespace

void setup()
{
    USB.begin(USB_BAUDRATE);
    UART.setTX(UART0_TX);
    UART.setRX(UART0_RX);
    UART.begin(UART_BAUDRATE);
}

void loop()
{
    poll(UART, uartReader, true);
    poll(USB, usbReader, false);
}

void setup1()
{
}

void loop1()
{
}
