#include <protocol.h>
#include <unity.h>

#include <cstdio>
#include <string>

using protocol::Command;
using protocol::CommandType;

void setUp() {}
void tearDown() {}

// CRC computed independently of the code under test
static std::string withCrc(const std::string &payload)
{
    unsigned sum = 0;
    for (unsigned char ch : payload)
        sum += ch;
    char crc[8];
    snprintf(crc, sizeof(crc), ",0x%02x", sum % 256);
    return payload + crc;
}

static bool parse(const std::string &line, bool requireCrc, Command &command)
{
    return protocol::parseCommand(line.c_str(), requireCrc, command);
}

void test_checksum_known_values()
{
    // 'A'+'T'+'+'+'V' = 278 = 0x116
    TEST_ASSERT_EQUAL_HEX8(0x16, protocol::checksum("AT+V", 4));
    TEST_ASSERT_EQUAL_HEX8(0x9d, protocol::checksum("AT+M,0,0,0,200,200", 18));
    TEST_ASSERT_EQUAL_HEX8(0x00, protocol::checksum("", 0));
}

void test_version_with_crc()
{
    Command command;
    TEST_ASSERT_TRUE(parse("AT+V,0x16", true, command));
    TEST_ASSERT_TRUE(command.type == CommandType::Version);
}

void test_wrong_crc_is_rejected_on_both_ports()
{
    Command command;
    TEST_ASSERT_FALSE(parse("AT+V,0x17", true, command));
    TEST_ASSERT_FALSE(parse("AT+V,0x17", false, command));
    TEST_ASSERT_TRUE(command.type == CommandType::None);
}

void test_missing_crc_only_accepted_if_not_required()
{
    Command command;
    TEST_ASSERT_FALSE(parse("AT+V", true, command));
    TEST_ASSERT_TRUE(parse("AT+V", false, command));
    TEST_ASSERT_TRUE(command.type == CommandType::Version);
}

void test_malformed_crc_is_rejected()
{
    Command command;
    TEST_ASSERT_FALSE(parse("AT+V,0x", false, command));
    TEST_ASSERT_FALSE(parse("AT+V,0x1g", false, command));
}

void test_motor_with_speed()
{
    Command command;
    TEST_ASSERT_TRUE(parse(withCrc("AT+M,10,-20,255,150,-300"), true, command));
    TEST_ASSERT_TRUE(command.type == CommandType::Motor);
    TEST_ASSERT_EQUAL_INT(10, command.rightPwm);
    TEST_ASSERT_EQUAL_INT(-20, command.leftPwm);
    TEST_ASSERT_EQUAL_INT(255, command.mowPwm);
    TEST_ASSERT_TRUE(command.hasSpeed);
    TEST_ASSERT_EQUAL_INT(150, command.rightSpeed);
    TEST_ASSERT_EQUAL_INT(-300, command.leftSpeed);
}

void test_motor_pwm_only()
{
    Command command;
    TEST_ASSERT_TRUE(parse(withCrc("AT+M,1,2,3"), true, command));
    TEST_ASSERT_EQUAL_INT(1, command.rightPwm);
    TEST_ASSERT_EQUAL_INT(2, command.leftPwm);
    TEST_ASSERT_EQUAL_INT(3, command.mowPwm);
    TEST_ASSERT_FALSE(command.hasSpeed);
}

void test_motor_with_wrong_field_count_is_rejected()
{
    Command command;
    TEST_ASSERT_FALSE(parse(withCrc("AT+M"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+M,1,2"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+M,1,2,3,4"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+M,1,2,3,4,5,6"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+M,1,2,3,4,5,6,7,8,9"), true, command));
}

void test_motor_with_invalid_number_is_rejected()
{
    Command command;
    TEST_ASSERT_FALSE(parse(withCrc("AT+M,1,x,3"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+M,1,,3"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+M,1,2,3,4.5,6"), true, command));
    TEST_ASSERT_TRUE(command.type == CommandType::None);
}

void test_summary_without_state()
{
    Command command;
    // what Sunray sends while messageToMcu is empty: 'A'+'T'+'+'+'S'+',' = 0x13f
    TEST_ASSERT_TRUE(parse("AT+S,,0x3f", true, command));
    TEST_ASSERT_TRUE(command.type == CommandType::Summary);
    TEST_ASSERT_FALSE(command.hasState);
    TEST_ASSERT_FALSE(command.hasGains);

    TEST_ASSERT_TRUE(parse("AT+S", false, command));
    TEST_ASSERT_FALSE(command.hasState);
}

void test_summary_with_state()
{
    Command command;
    TEST_ASSERT_TRUE(parse(withCrc("AT+S,4"), true, command));
    TEST_ASSERT_TRUE(command.hasState);
    TEST_ASSERT_EQUAL_INT(4, command.state);
    TEST_ASSERT_FALSE(command.hasGains);

    TEST_ASSERT_TRUE(parse(withCrc("AT+S,-1"), true, command));
    TEST_ASSERT_EQUAL_INT(-1, command.state);
}

void test_summary_with_gains()
{
    Command command;
    TEST_ASSERT_TRUE(parse(withCrc("AT+S,1,8.00,0.10,0.01"), true, command));
    TEST_ASSERT_TRUE(command.hasState);
    TEST_ASSERT_EQUAL_INT(1, command.state);
    TEST_ASSERT_TRUE(command.hasGains);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 8.0f, command.kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, command.ki);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, command.kd);
}

void test_summary_with_incomplete_gains_is_rejected()
{
    Command command;
    TEST_ASSERT_FALSE(parse(withCrc("AT+S,1,8.00"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+S,1,8.00,0.10"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+S,1,8.00,abc,0.01"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+S,idle"), true, command));
}

void test_commands_without_arguments()
{
    Command command;
    TEST_ASSERT_TRUE(parse(withCrc("AT+R"), true, command));
    TEST_ASSERT_TRUE(command.type == CommandType::ResetFaults);
    TEST_ASSERT_TRUE(parse(withCrc("AT+E"), true, command));
    TEST_ASSERT_TRUE(command.type == CommandType::MotorTest);
    TEST_ASSERT_TRUE(parse(withCrc("AT+Y"), true, command));
    TEST_ASSERT_TRUE(command.type == CommandType::TriggerWatchdog);
    TEST_ASSERT_TRUE(parse(withCrc("AT+Y3"), true, command));
    TEST_ASSERT_TRUE(command.type == CommandType::Shutdown);
}

void test_unknown_input_is_rejected()
{
    Command command;
    TEST_ASSERT_FALSE(parse("", false, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+X"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+V,1"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+Y3,1"), true, command));
    TEST_ASSERT_FALSE(parse("hello", false, command));
    TEST_ASSERT_FALSE(parse(std::string(protocol::MAX_LINE, 'A'), false, command));
}

void test_failed_parse_clears_previous_command()
{
    Command command;
    TEST_ASSERT_TRUE(parse(withCrc("AT+M,1,2,3,4,5"), true, command));
    TEST_ASSERT_FALSE(parse(withCrc("AT+M,1,2,x"), true, command));
    TEST_ASSERT_TRUE(command.type == CommandType::None);
    TEST_ASSERT_EQUAL_INT(0, command.rightPwm);
    TEST_ASSERT_FALSE(command.hasSpeed);
}

void test_format_frame()
{
    char out[32];
    TEST_ASSERT_EQUAL_UINT(8, protocol::formatFrame(out, sizeof(out), "R"));
    TEST_ASSERT_EQUAL_STRING("R,0x52\r\n", out);
    // crc below 0x10 keeps two digits: '\t' = 0x09
    protocol::formatFrame(out, sizeof(out), "\t");
    TEST_ASSERT_EQUAL_STRING("\t,0x09\r\n", out);
}

void test_format_version()
{
    char out[protocol::MAX_LINE];
    const size_t length = protocol::formatVersion(out, sizeof(out), "Landrumower RPI Pico", "4.0.0");
    const std::string expected = withCrc("V,Landrumower RPI Pico,4.0.0") + "\r\n";
    TEST_ASSERT_EQUAL_STRING(expected.c_str(), out);
    TEST_ASSERT_EQUAL_UINT(expected.size(), length);
}

void test_format_motor()
{
    protocol::MotorReport report;
    report.rightTicks = 1234;
    report.leftTicks = -56;
    report.mowTicks = 7;
    report.chargeVoltage = 28.456f;
    report.bumper = true;
    report.lift = false;
    report.stopButton = true;
    char out[protocol::MAX_LINE];
    protocol::formatMotor(out, sizeof(out), report);
    const std::string expected = withCrc("M,1234,-56,7,28.46,1,0,1") + "\r\n";
    TEST_ASSERT_EQUAL_STRING(expected.c_str(), out);
}

void test_format_summary()
{
    protocol::SummaryReport report;
    report.batteryVoltage = 25.2f;
    report.chargeVoltage = 0.0f;
    report.chargeCurrent = 1.5f;
    report.lift = false;
    report.bumper = true;
    report.rain = false;
    report.motorFault = true;
    report.mowCurrent = 2.25f;
    report.leftCurrent = 0.5f;
    report.rightCurrent = 0.75f;
    report.batteryTemperature = 21.0f;
    char out[protocol::MAX_LINE];
    protocol::formatSummary(out, sizeof(out), report);
    const std::string expected = withCrc("S,25.20,0.00,1.50,0,1,0,1,2.25,0.50,0.75,21.00") + "\r\n";
    TEST_ASSERT_EQUAL_STRING(expected.c_str(), out);
}

void test_format_into_too_small_buffer_returns_zero()
{
    char out[8];
    // "R,0x52\r\n" needs 8 characters plus the terminator
    TEST_ASSERT_EQUAL_UINT(0, protocol::formatFrame(out, sizeof(out), "R"));
    TEST_ASSERT_EQUAL_STRING("", out);
    TEST_ASSERT_EQUAL_UINT(0, protocol::formatMotor(out, sizeof(out), protocol::MotorReport()));
}

void test_formatted_response_parses_as_valid_frame()
{
    // what the Pico sends must pass the CRC check Sunray applies
    char out[protocol::MAX_LINE];
    const size_t length = protocol::formatSummary(out, sizeof(out), protocol::SummaryReport());
    const std::string frame(out, length - 2);
    const size_t comma = frame.rfind(',');
    const unsigned received = std::stoul(frame.substr(comma + 1), nullptr, 16);
    TEST_ASSERT_EQUAL_HEX8(protocol::checksum(frame.c_str(), comma), received);
}

static bool feedAll(protocol::LineReader &reader, const std::string &text)
{
    bool complete = false;
    for (char ch : text)
        complete = reader.feed(ch);
    return complete;
}

void test_line_reader_assembles_line()
{
    protocol::LineReader reader;
    TEST_ASSERT_FALSE(feedAll(reader, "AT+V,0x16"));
    TEST_ASSERT_TRUE(reader.feed('\r'));
    TEST_ASSERT_EQUAL_STRING("AT+V,0x16", reader.line());
    // the '\n' of "\r\n" must not produce an empty line
    TEST_ASSERT_FALSE(reader.feed('\n'));
}

void test_line_reader_handles_consecutive_lines()
{
    protocol::LineReader reader;
    TEST_ASSERT_TRUE(feedAll(reader, "first\r\nsecond\n"));
    TEST_ASSERT_EQUAL_STRING("second", reader.line());
    TEST_ASSERT_TRUE(feedAll(reader, "third\r"));
    TEST_ASSERT_EQUAL_STRING("third", reader.line());
}

void test_line_reader_drops_overlong_line()
{
    protocol::LineReader reader;
    TEST_ASSERT_FALSE(feedAll(reader, std::string(protocol::MAX_LINE + 10, 'x') + "\r\n"));
    // and recovers with the next line
    TEST_ASSERT_TRUE(feedAll(reader, "AT+V\r"));
    TEST_ASSERT_EQUAL_STRING("AT+V", reader.line());
}

void test_line_reader_accepts_longest_line()
{
    protocol::LineReader reader;
    const std::string longest(protocol::MAX_LINE - 1, 'x');
    TEST_ASSERT_TRUE(feedAll(reader, longest + "\n"));
    TEST_ASSERT_EQUAL_STRING(longest.c_str(), reader.line());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_checksum_known_values);
    RUN_TEST(test_version_with_crc);
    RUN_TEST(test_wrong_crc_is_rejected_on_both_ports);
    RUN_TEST(test_missing_crc_only_accepted_if_not_required);
    RUN_TEST(test_malformed_crc_is_rejected);
    RUN_TEST(test_motor_with_speed);
    RUN_TEST(test_motor_pwm_only);
    RUN_TEST(test_motor_with_wrong_field_count_is_rejected);
    RUN_TEST(test_motor_with_invalid_number_is_rejected);
    RUN_TEST(test_summary_without_state);
    RUN_TEST(test_summary_with_state);
    RUN_TEST(test_summary_with_gains);
    RUN_TEST(test_summary_with_incomplete_gains_is_rejected);
    RUN_TEST(test_commands_without_arguments);
    RUN_TEST(test_unknown_input_is_rejected);
    RUN_TEST(test_failed_parse_clears_previous_command);
    RUN_TEST(test_format_frame);
    RUN_TEST(test_format_version);
    RUN_TEST(test_format_motor);
    RUN_TEST(test_format_summary);
    RUN_TEST(test_format_into_too_small_buffer_returns_zero);
    RUN_TEST(test_formatted_response_parses_as_valid_frame);
    RUN_TEST(test_line_reader_assembles_line);
    RUN_TEST(test_line_reader_handles_consecutive_lines);
    RUN_TEST(test_line_reader_drops_overlong_line);
    RUN_TEST(test_line_reader_accepts_longest_line);
    return UNITY_END();
}
