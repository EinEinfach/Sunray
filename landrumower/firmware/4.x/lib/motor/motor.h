// Control of one motor: speed control or PWM pass-through, stopping, braking, reversing.
//
// MotorController is pure logic. It turns a demand and the measured speed into a MotorOutput.
// MotorAxis connects a controller to an encoder and a driver, real or simulated.

#ifndef MOTOR_H
#define MOTOR_H

#include <encoder.h>
#include <pid.h>
#include <stdint.h>

struct MotorOutput
{
    bool forward = true;
    float drive = 0.0f; // duty 0..1 on the speed input of the driver
    float brake = 0.0f; // duty 0..1 on the brake input of the driver
};

class MotorDriver
{
public:
    virtual ~MotorDriver() = default;
    virtual void apply(const MotorOutput &output) = 0;
};

struct MotorConfig
{
    PidGains gains; // output is duty per speed unit

    // Braking with PWM on the brake input. The usable duty range of the driver is unknown
    // until measured, so it is configurable. If disabled, the brake is only switched on or off.
    bool pwmBraking = false;
    float brakeMinDuty = 0.0f;
    float brakeMaxDuty = 1.0f;

    float standstillSpeed = 0.0f;  // at or below this speed the motor counts as stopped
    float stopBrakeDelay = 0.05f;  // seconds to coast before the brake is applied on stop
    float stopBrakeRamp = 0.5f;    // seconds to ramp the brake to full on stop
    float commandTimeout = 3.0f;   // seconds without a new demand until the motor stops
};

class MotorController
{
public:
    explicit MotorController(const MotorConfig &config);

    // closed loop, signed speed in the unit of the measured speed
    void setSpeed(float speed);
    // open loop, -255..255 as sent by Sunray
    void setPwm(int pwm);
    void setGains(PidGains gains);

    // measuredSpeed is unsigned, because the encoder has no direction. dt in seconds.
    const MotorOutput &update(float measuredSpeed, float dt);

    float demand() const { return target; }

private:
    enum class Mode
    {
        Speed,
        Pwm
    };

    void setDemand(Mode newMode, float value);
    void stop(bool turning, float dt);

    MotorConfig config;
    Pid pid;
    Mode mode = Mode::Speed;
    float target = 0.0f;
    float commandAge = 0.0f;
    float stopTime = 0.0f;
    float brakeTime = 0.0f;
    bool braking = false;
    MotorOutput output;
};

class MotorAxis
{
public:
    // unitsPerTick converts encoder impulses to the speed unit of the controller
    MotorAxis(EncoderSource &encoder, MotorDriver &driver, const MotorConfig &config,
              float unitsPerTick, uint32_t standstillTimeoutMicros);

    void update(uint32_t nowMicros, float dt);

    MotorController &control() { return controller; }
    uint32_t ticks() const { return tickCount; }
    float speed() const { return measuredSpeed; }
    const MotorOutput &output() const { return lastOutput; }

private:
    EncoderSource &encoder;
    MotorDriver &driver;
    SpeedEstimator estimator;
    MotorController controller;
    float unitsPerTick;
    uint32_t tickCount = 0;
    float measuredSpeed = 0.0f;
    MotorOutput lastOutput;
};

#endif // MOTOR_H
