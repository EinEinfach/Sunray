#include "motor.h"

#include <math.h>

MotorController::MotorController(const MotorConfig &config)
    : config(config), pid(config.gains, config.pwmBraking ? -1.0f : 0.0f, 1.0f)
{
}

void MotorController::setSpeed(float speed)
{
    setDemand(Mode::Speed, speed);
}

void MotorController::setPwm(int pwm)
{
    if (pwm > 255)
        pwm = 255;
    else if (pwm < -255)
        pwm = -255;
    setDemand(Mode::Pwm, static_cast<float>(pwm));
}

void MotorController::setGains(PidGains gains)
{
    pid.setGains(gains);
}

void MotorController::setDemand(Mode newMode, float value)
{
    if (newMode != mode)
    {
        mode = newMode;
        pid.reset();
    }
    target = value;
    commandAge = 0.0f;
}

const MotorOutput &MotorController::update(float measuredSpeed, float dt)
{
    commandAge += dt;
    if (commandAge > config.commandTimeout)
        target = 0.0f;

    const bool wantForward = target >= 0.0f;
    const float magnitude = fabsf(target);
    const bool turning = measuredSpeed > config.standstillSpeed;

    if (magnitude == 0.0f)
    {
        stop(turning, dt);
        return output;
    }
    // the driver must not be reversed while the motor is turning
    if ((wantForward != output.forward) && turning)
    {
        stop(turning, dt);
        return output;
    }
    if (wantForward != output.forward)
    {
        output.forward = wantForward;
        pid.reset();
    }
    stopTime = 0.0f;
    brakeTime = 0.0f;
    braking = false;

    if (mode == Mode::Pwm)
    {
        output.drive = magnitude / 255.0f;
        output.brake = 0.0f;
        return output;
    }

    const float effort = pid.update(magnitude, measuredSpeed, dt);
    if (effort >= 0.0f)
    {
        output.drive = effort;
        output.brake = 0.0f;
    }
    else
    {
        output.drive = 0.0f;
        output.brake = config.brakeMinDuty - effort * (config.brakeMaxDuty - config.brakeMinDuty);
    }
    return output;
}

void MotorController::stop(bool turning, float dt)
{
    pid.reset();
    output.drive = 0.0f;
    stopTime += dt;

    // coast first, then brake if the motor is still turning. Once applied the brake holds.
    if (!braking && turning && (stopTime >= config.stopBrakeDelay))
        braking = true;
    if (!braking)
    {
        output.brake = 0.0f;
        return;
    }
    brakeTime += dt;
    if (!config.pwmBraking || (config.stopBrakeRamp <= 0.0f) || (brakeTime >= config.stopBrakeRamp))
        output.brake = 1.0f;
    else
        output.brake = brakeTime / config.stopBrakeRamp;
}

MotorAxis::MotorAxis(EncoderSource &encoder, MotorDriver &driver, const MotorConfig &config,
                     float unitsPerTick, uint32_t standstillTimeoutMicros)
    : encoder(encoder), driver(driver), estimator(standstillTimeoutMicros), controller(config),
      unitsPerTick(unitsPerTick)
{
}

void MotorAxis::update(uint32_t nowMicros, float dt)
{
    const EncoderSample sample = encoder.sample();
    tickCount = sample.ticks;
    measuredSpeed = estimator.update(sample, nowMicros) * unitsPerTick;
    lastOutput = controller.update(measuredSpeed, dt);
    driver.apply(lastOutput);
}
