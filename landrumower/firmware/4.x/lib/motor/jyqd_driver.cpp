#ifdef ARDUINO

#include "jyqd_driver.h"

#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>

namespace
{

// Sets up the PWM slice of a pin and returns the counter value for 100 % duty.
// The Pico SDK is used directly, because analogWriteFreq() of the Arduino core is global.
uint32_t configurePwm(uint8_t pin, uint32_t hz)
{
    const uint32_t clock = clock_get_hz(clk_sys);
    uint32_t divider = 1;
    while (clock / (hz * divider) > 65536)
        divider++;
    const uint32_t top = clock / (hz * divider);

    const uint slice = pwm_gpio_to_slice_num(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_clkdiv_int_frac(slice, static_cast<uint8_t>(divider), 0);
    pwm_set_wrap(slice, static_cast<uint16_t>(top - 1));
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice, true);
    return top;
}

void setDuty(uint8_t pin, uint32_t top, float duty)
{
    if (duty < 0.0f)
        duty = 0.0f;
    else if (duty > 1.0f)
        duty = 1.0f;
    pwm_set_gpio_level(pin, static_cast<uint16_t>(duty * static_cast<float>(top)));
}

} // namespace

JyqdDriver::JyqdDriver(const JyqdPins &pins, uint32_t pwmHz, uint32_t brakeHz)
    : pins(pins), pwmHz(pwmHz), brakeHz(brakeHz)
{
}

void JyqdDriver::begin()
{
    pinMode(pins.direction, OUTPUT);
    pwmTop = configurePwm(pins.pwm, pwmHz);
    brakeTop = configurePwm(pins.brake, brakeHz);
    apply(MotorOutput());
}

void JyqdDriver::apply(const MotorOutput &output)
{
    digitalWrite(pins.direction, output.forward == pins.directionHighIsForward ? HIGH : LOW);
    setDuty(pins.pwm, pwmTop, output.drive);
    setDuty(pins.brake, brakeTop, pins.brakeHighActive ? output.brake : 1.0f - output.brake);
}

#endif // ARDUINO
