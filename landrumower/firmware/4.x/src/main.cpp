#include <Arduino.h>
#include <Wire.h>
#include <atomic>
#include <buzzer.h>
#include <lcd.h>
#include <motor.h>
#include <mower_state.h>
#include <pcf8574_lcd.h>
#include <pico/mutex.h>
#include <protocol.h>
#include "config.h"

#ifdef SIMULATED_HARDWARE
#include <motor_sim.h>
#else
#include <jyqd_driver.h>
#include <pin_encoder.h>
#endif

// Core 0: Sunray link and motor control loop.
// Core 1: slow and blocking I/O: LCD, buzzer and the output on the USB console.

namespace
{

constexpr float MM_PER_TICK = 3.14159265f * WHEEL_DIAMETER_MM / TICKS_PER_REVOLUTION;

MotorConfig motorConfig(float standstillSpeed)
{
    MotorConfig config;
    config.gains = {MOTOR_KP, MOTOR_KI, MOTOR_KD};
    config.pwmBraking = BRAKE_PWM;
    config.brakeMinDuty = BRAKE_MIN_DUTY;
    config.brakeMaxDuty = BRAKE_MAX_DUTY;
    config.standstillSpeed = standstillSpeed;
    config.stopBrakeDelay = STOP_BRAKE_DELAY_S;
    config.stopBrakeRamp = STOP_BRAKE_RAMP_S;
    config.commandTimeout = MOTOR_COMMAND_TIMEOUT_S;
    return config;
}

#ifdef SIMULATED_HARDWARE

MotorSimConfig simConfig(float maxTicksPerSecond)
{
    MotorSimConfig config;
    config.maxTicksPerSecond = maxTicksPerSecond;
    return config;
}

MotorSim rightSim(simConfig(SIM_WHEEL_MAX_TICKS_S));
MotorSim leftSim(simConfig(SIM_WHEEL_MAX_TICKS_S));
MotorSim mowSim(simConfig(SIM_MOW_MAX_TICKS_S));

MotorAxis rightMotor(rightSim, rightSim, motorConfig(STANDSTILL_SPEED_MM_S), MM_PER_TICK, STANDSTILL_TIMEOUT_US);
MotorAxis leftMotor(leftSim, leftSim, motorConfig(STANDSTILL_SPEED_MM_S), MM_PER_TICK, STANDSTILL_TIMEOUT_US);
MotorAxis mowMotor(mowSim, mowSim, motorConfig(MOW_STANDSTILL_TICKS_S), 1.0f, STANDSTILL_TIMEOUT_US);

void setupMotors()
{
}

void stepHardware(uint32_t now)
{
    rightSim.step(now);
    leftSim.step(now);
    mowSim.step(now);
}

#else

JyqdDriver rightDriver({RIGHT_PWM, RIGHT_DIR, RIGHT_BRAKE, RIGHT_DIRECTION_HIGH_IS_FORWARD, RIGHT_BRAKE_HIGH_ACTIVE}, MOTOR_PWM_HZ, BRAKE_PWM_HZ);
JyqdDriver leftDriver({LEFT_PWM, LEFT_DIR, LEFT_BRAKE, LEFT_DIRECTION_HIGH_IS_FORWARD, LEFT_BRAKE_HIGH_ACTIVE}, MOTOR_PWM_HZ, BRAKE_PWM_HZ);
JyqdDriver mowDriver({MOW_PWM, MOW_DIR, MOW_BRAKE, MOW_DIRECTION_HIGH_IS_FORWARD, MOW_BRAKE_HIGH_ACTIVE}, MOTOR_PWM_HZ, BRAKE_PWM_HZ);

PinEncoder rightEncoder(RIGHT_IMP, ENCODER_MIN_EDGE_US);
PinEncoder leftEncoder(LEFT_IMP, ENCODER_MIN_EDGE_US);
PinEncoder mowEncoder(MOW_IMP, ENCODER_MIN_EDGE_US);

MotorAxis rightMotor(rightEncoder, rightDriver, motorConfig(STANDSTILL_SPEED_MM_S), MM_PER_TICK, STANDSTILL_TIMEOUT_US);
MotorAxis leftMotor(leftEncoder, leftDriver, motorConfig(STANDSTILL_SPEED_MM_S), MM_PER_TICK, STANDSTILL_TIMEOUT_US);
MotorAxis mowMotor(mowEncoder, mowDriver, motorConfig(MOW_STANDSTILL_TICKS_S), 1.0f, STANDSTILL_TIMEOUT_US);

void setupMotors()
{
    rightDriver.begin();
    leftDriver.begin();
    mowDriver.begin();
    rightEncoder.begin();
    leftEncoder.begin();
    mowEncoder.begin();
}

void stepHardware(uint32_t)
{
}

#endif

// ---------------------------------------------------------------- state shared with core 1

struct AxisState
{
    float demand = 0.0f;
    float speed = 0.0f;
    float drive = 0.0f; // negative when driving backwards
    float brake = 0.0f;
};

struct Telemetry
{
    AxisState right;
    AxisState left;
    AxisState mow;
    uint32_t maxPeriodMicros = 0; // longest control period since the last telemetry line
};

auto_init_mutex(telemetryMutex);
Telemetry telemetry;

// written by core 0 when Sunray reports its state
std::atomic<MowerState> mowerState{MowerState::Boot};

AxisState stateOf(MotorAxis &axis)
{
    AxisState state;
    state.demand = axis.control().demand();
    state.speed = axis.speed();
    state.drive = axis.output().forward ? axis.output().drive : -axis.output().drive;
    state.brake = axis.output().brake;
    return state;
}

// ---------------------------------------------------------------- core 0

protocol::LineReader uartReader;
protocol::LineReader usbReader;
uint32_t lastControlMicros = 0;
uint32_t pendingMaxPeriodMicros = 0;

void handleMotor(const protocol::Command &command)
{
    if (PICO_MOTOR_CONTROL && command.hasSpeed)
    {
        rightMotor.control().setSpeed(static_cast<float>(command.rightSpeed));
        leftMotor.control().setSpeed(static_cast<float>(command.leftSpeed));
    }
    else
    {
        rightMotor.control().setPwm(command.rightPwm);
        leftMotor.control().setPwm(command.leftPwm);
    }
    mowMotor.control().setPwm(command.mowPwm);
}

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
    case protocol::CommandType::Motor:
    {
        handleMotor(command);
        // charge voltage and sensors are not implemented yet
        protocol::MotorReport report;
        report.rightTicks = static_cast<long>(rightMotor.ticks());
        report.leftTicks = static_cast<long>(leftMotor.ticks());
        report.mowTicks = static_cast<long>(mowMotor.ticks());
        length = protocol::formatMotor(response, sizeof(response), report);
        break;
    }
    case protocol::CommandType::Summary:
        if (command.hasState)
            mowerState.store(mowerStateFromSunray(command.state));
        if (command.hasGains)
        {
            rightMotor.control().setGains({command.kp, command.ki, command.kd});
            leftMotor.control().setGains({command.kp, command.ki, command.kd});
        }
        // battery, currents and sensors are not implemented yet
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

void control(uint32_t now, uint32_t elapsedMicros)
{
    const float dt = static_cast<float>(elapsedMicros) * 1e-6f;
    stepHardware(now);
    rightMotor.update(now, dt);
    leftMotor.update(now, dt);
    mowMotor.update(now, dt);

    if (elapsedMicros > pendingMaxPeriodMicros)
        pendingMaxPeriodMicros = elapsedMicros;
    // never wait for core 1, the next control step publishes again
    if (mutex_try_enter(&telemetryMutex, nullptr))
    {
        telemetry.right = stateOf(rightMotor);
        telemetry.left = stateOf(leftMotor);
        telemetry.mow = stateOf(mowMotor);
        if (pendingMaxPeriodMicros > telemetry.maxPeriodMicros)
            telemetry.maxPeriodMicros = pendingMaxPeriodMicros;
        pendingMaxPeriodMicros = 0;
        mutex_exit(&telemetryMutex);
    }
}

// ---------------------------------------------------------------- core 1

void printTelemetry()
{
    mutex_enter_blocking(&telemetryMutex);
    const Telemetry snapshot = telemetry;
    telemetry.maxPeriodMicros = 0;
    mutex_exit(&telemetryMutex);

    char line[160];
    const int length = snprintf(line, sizeof(line),
                                "D,%lu,%.0f,%.1f,%.3f,%.3f,%.0f,%.1f,%.3f,%.3f,%.0f,%.1f,%.3f,%.3f,%lu\r\n",
                                static_cast<unsigned long>(millis()),
                                snapshot.right.demand, snapshot.right.speed, snapshot.right.drive, snapshot.right.brake,
                                snapshot.left.demand, snapshot.left.speed, snapshot.left.drive, snapshot.left.brake,
                                snapshot.mow.demand, snapshot.mow.speed, snapshot.mow.drive, snapshot.mow.brake,
                                static_cast<unsigned long>(snapshot.maxPeriodMicros));
    if ((length > 0) && (static_cast<size_t>(length) < sizeof(line)))
        USB.write(line, static_cast<size_t>(length));
}

BuzzerPlayer buzzerPlayer;
bool buzzerOn = false;

void updateBuzzer(MowerState state, uint32_t now)
{
    buzzerPlayer.setState(state, now);
    buzzerOn = buzzerPlayer.update(now);
    if (BUZZER_ENABLED)
        digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
}

LcdScreen screen;
Pcf8574Lcd display(Wire1, LCD_ADDRESS);
bool lcdPresent = false;
bool frameShown = false;
lcd::Frame shownFrame;
uint32_t lastLcdRetryMillis = 0;

void updateLcd(MowerState state, uint32_t now)
{
    // the start screen stays until Sunray reports its state. The second line is reserved
    // for the battery, which is not implemented yet.
    if (state != MowerState::Boot)
    {
        screen.setLine(0, mowerStateText(state));
        screen.setLine(1, "");
    }
    if (!lcdPresent)
    {
        if (now - lastLcdRetryMillis < LCD_RETRY_MS)
            return;
        lastLcdRetryMillis = now;
        lcdPresent = display.begin();
        frameShown = false;
        if (!lcdPresent)
            return;
    }
    const lcd::Frame frame = screen.frame(now);
    if (frameShown && (frame == shownFrame))
        return;
    if (display.show(frame))
    {
        shownFrame = frame;
        frameShown = true;
    }
    else
    {
        lcdPresent = false;
        lastLcdRetryMillis = now;
    }
}

void printInfo(MowerState state)
{
    char line[80];
    const int length = snprintf(line, sizeof(line), "I,state=%s,lcd=%d,buzzer=%d\r\n",
                                mowerStateText(state), lcdPresent, buzzerOn);
    if ((length > 0) && (static_cast<size_t>(length) < sizeof(line)))
        USB.write(line, static_cast<size_t>(length));
}

// true once per period
bool due(uint32_t &last, uint32_t now, uint32_t period)
{
    if ((period == 0) || (now - last < period))
        return false;
    last = now;
    return true;
}

} // namespace

void setup()
{
    USB.begin(USB_BAUDRATE);
    setupMotors();
    // The UART is set up last. On the RP2040 a later pinMode() for another component
    // could break its pin configuration (finding from firmware 3.x).
    UART.setTX(UART0_TX);
    UART.setRX(UART0_RX);
    UART.begin(UART_BAUDRATE);
    lastControlMicros = micros();
}

void loop()
{
    poll(UART, uartReader, true);
    poll(USB, usbReader, false);

    const uint32_t now = micros();
    const uint32_t elapsed = now - lastControlMicros;
    if (elapsed >= CONTROL_PERIOD_US)
    {
        lastControlMicros = now;
        control(now, elapsed);
    }
}

void setup1()
{
    if (BUZZER_ENABLED)
    {
        pinMode(PIN_BUZZER, OUTPUT);
        digitalWrite(PIN_BUZZER, LOW);
    }
    screen.setLine(0, "Landrumower pico");
    screen.setLine(1, "Version: " FW_VERSION);
    if (LCD_ENABLED)
    {
        Wire1.setSDA(I2C1_SDA);
        Wire1.setSCL(I2C1_SCL);
        Wire1.setClock(I2C1_CLOCK_HZ);
        Wire1.begin();
        lcdPresent = display.begin();
        lastLcdRetryMillis = millis();
    }
}

void loop1()
{
    static uint32_t lastLcdMillis = 0;
    static uint32_t lastTelemetryMillis = 0;
    static uint32_t lastInfoMillis = 0;

    const uint32_t now = millis();
    const MowerState state = mowerState.load();

    updateBuzzer(state, now);
    if (LCD_ENABLED && due(lastLcdMillis, now, 200))
        updateLcd(state, now);

    const bool telemetryDue = due(lastTelemetryMillis, now, TELEMETRY_PERIOD_MS);
    const bool infoDue = due(lastInfoMillis, now, INFO_PERIOD_MS);
    if ((telemetryDue || infoDue) && USB)
    {
        if (telemetryDue)
            printTelemetry();
        if (infoDue)
            printInfo(state);
    }
    delay(5);
}
