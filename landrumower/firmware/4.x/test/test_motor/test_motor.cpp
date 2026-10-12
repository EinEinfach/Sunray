#include <motor.h>
#include <motor_sim.h>
#include <unity.h>

#include <cmath>

void setUp() {}
void tearDown() {}

static const float DT = 0.005f;
static const uint32_t STEP = 5000;
static const float TOLERANCE = 1e-4f;

// ---------------------------------------------------------------- MotorController

static MotorConfig proportionalConfig()
{
    MotorConfig config;
    config.gains = {0.001f, 0.0f, 0.0f};
    config.pwmBraking = true;
    config.brakeMinDuty = 0.1f;
    config.brakeMaxDuty = 0.6f;
    config.standstillSpeed = 20.0f;
    config.stopBrakeDelay = 0.05f;
    config.stopBrakeRamp = 0.5f;
    config.commandTimeout = 3.0f;
    return config;
}

static MotorOutput runFor(MotorController &controller, float measuredSpeed, float seconds)
{
    MotorOutput output;
    const int steps = static_cast<int>(lroundf(seconds / DT));
    for (int i = 0; i < steps; i++)
        output = controller.update(measuredSpeed, DT);
    return output;
}

void test_pwm_mode_passes_duty_through()
{
    MotorController controller(proportionalConfig());
    controller.setPwm(128);
    MotorOutput output = controller.update(0.0f, DT);
    TEST_ASSERT_TRUE(output.forward);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 128.0f / 255.0f, output.drive);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.brake);

    // the measured speed has no influence
    output = controller.update(500.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 128.0f / 255.0f, output.drive);
}

void test_pwm_mode_backwards_and_clamped()
{
    MotorController controller(proportionalConfig());
    controller.setPwm(-400);
    const MotorOutput output = controller.update(0.0f, DT);
    TEST_ASSERT_FALSE(output.forward);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 1.0f, output.drive);
}

void test_speed_mode_drives_towards_setpoint()
{
    MotorController controller(proportionalConfig());
    controller.setSpeed(300.0f);
    const MotorOutput output = controller.update(100.0f, DT);
    TEST_ASSERT_TRUE(output.forward);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.2f, output.drive);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.brake);
}

void test_speed_mode_backwards_uses_unsigned_speed()
{
    MotorController controller(proportionalConfig());
    controller.setSpeed(-300.0f);
    controller.update(0.0f, DT);
    const MotorOutput output = controller.update(100.0f, DT);
    TEST_ASSERT_FALSE(output.forward);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.2f, output.drive);
}

void test_too_fast_brakes_within_configured_range()
{
    MotorController controller(proportionalConfig());
    controller.setSpeed(300.0f);
    // effort -0.2 maps into the brake range 0.1..0.6
    MotorOutput output = controller.update(500.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.drive);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.2f, output.brake);

    // effort is limited to -1, so the brake never exceeds its maximum
    output = controller.update(5000.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.6f, output.brake);
}

void test_too_fast_coasts_without_pwm_braking()
{
    MotorConfig config = proportionalConfig();
    config.pwmBraking = false;
    MotorController controller(config);
    controller.setSpeed(300.0f);
    const MotorOutput output = controller.update(500.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.drive);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.brake);
}

void test_stop_coasts_then_ramps_brake_and_holds()
{
    MotorController controller(proportionalConfig());
    controller.setSpeed(300.0f);
    controller.update(300.0f, DT);
    controller.setSpeed(0.0f);

    MotorOutput output = runFor(controller, 200.0f, 0.045f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.drive);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.brake);

    output = runFor(controller, 200.0f, 0.255f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.drive);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, output.brake);

    output = runFor(controller, 200.0f, 0.3f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, output.brake);

    // the brake holds after the motor has stopped
    output = runFor(controller, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, output.brake);
}

void test_stop_without_pwm_braking_switches_brake_on()
{
    MotorConfig config = proportionalConfig();
    config.pwmBraking = false;
    MotorController controller(config);
    controller.setSpeed(0.0f);
    MotorOutput output = runFor(controller, 200.0f, 0.045f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.brake);
    output = runFor(controller, 200.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, output.brake);
}

void test_brake_is_not_applied_to_a_motor_at_rest()
{
    MotorController controller(proportionalConfig());
    const MotorOutput output = runFor(controller, 0.0f, 2.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.drive);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.brake);
}

void test_brake_releases_when_driving_again()
{
    MotorController controller(proportionalConfig());
    runFor(controller, 200.0f, 1.0f);
    controller.setSpeed(300.0f);
    const MotorOutput output = controller.update(0.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.brake);
    TEST_ASSERT_TRUE(output.drive > 0.0f);
}

void test_reversing_stops_the_motor_first()
{
    MotorController controller(proportionalConfig());
    controller.setSpeed(300.0f);
    controller.update(200.0f, DT);

    controller.setSpeed(-300.0f);
    MotorOutput output = runFor(controller, 200.0f, 0.3f);
    TEST_ASSERT_TRUE(output.forward);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.drive);
    TEST_ASSERT_TRUE(output.brake > 0.0f);

    // at standstill the direction changes and the brake is released
    output = controller.update(0.0f, DT);
    TEST_ASSERT_FALSE(output.forward);
    TEST_ASSERT_TRUE(output.drive > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.brake);
}

void test_missing_commands_stop_the_motor()
{
    MotorController controller(proportionalConfig());
    controller.setSpeed(300.0f);
    MotorOutput output = runFor(controller, 100.0f, 2.9f);
    TEST_ASSERT_TRUE(output.drive > 0.0f);
    output = runFor(controller, 100.0f, 0.2f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.drive);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, controller.demand());
}

void test_repeated_commands_keep_the_motor_running()
{
    MotorController controller(proportionalConfig());
    MotorOutput output;
    for (int i = 0; i < 10; i++)
    {
        controller.setPwm(100);
        output = runFor(controller, 100.0f, 1.0f);
    }
    TEST_ASSERT_TRUE(output.drive > 0.0f);
}

void test_gains_can_be_changed()
{
    MotorController controller(proportionalConfig());
    controller.setGains({0.002f, 0.0f, 0.0f});
    controller.setSpeed(300.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.4f, controller.update(100.0f, DT).drive);
}

// ---------------------------------------------------------------- MotorSim

static void stepSim(MotorSim &sim, uint32_t &now, float seconds)
{
    const int steps = static_cast<int>(lroundf(seconds / DT));
    for (int i = 0; i < steps; i++)
    {
        now += STEP;
        sim.step(now);
    }
}

static MotorOutput driveOutput(bool forward, float drive, float brake)
{
    MotorOutput output;
    output.forward = forward;
    output.drive = drive;
    output.brake = brake;
    return output;
}

void test_sim_reaches_speed_proportional_to_drive()
{
    MotorSim sim{MotorSimConfig()};
    uint32_t now = 0;
    sim.step(now);
    sim.apply(driveOutput(true, 1.0f, 0.0f));
    stepSim(sim, now, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 350.0f, sim.velocity());

    sim.apply(driveOutput(true, 0.5f, 0.0f));
    stepSim(sim, now, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 175.0f, sim.velocity());
}

void test_sim_counts_impulses_in_both_directions()
{
    MotorSim sim{MotorSimConfig()};
    uint32_t now = 0;
    sim.step(now);
    sim.apply(driveOutput(false, 1.0f, 0.0f));
    stepSim(sim, now, 2.0f);
    TEST_ASSERT_TRUE(sim.velocity() < -340.0f);

    const uint32_t before = sim.sample().ticks;
    stepSim(sim, now, 1.0f);
    TEST_ASSERT_UINT32_WITHIN(3, 350, sim.sample().ticks - before);
}

void test_sim_impulse_time_is_never_in_the_future()
{
    MotorSim sim{MotorSimConfig()};
    uint32_t now = 0;
    sim.step(now);
    sim.apply(driveOutput(true, 0.7f, 0.0f));
    uint32_t lastEdge = 0;
    for (int i = 0; i < 400; i++)
    {
        now += STEP;
        sim.step(now);
        const EncoderSample sample = sim.sample();
        TEST_ASSERT_TRUE(sample.lastEdgeMicros <= now);
        TEST_ASSERT_TRUE(sample.lastEdgeMicros >= lastEdge);
        lastEdge = sample.lastEdgeMicros;
    }
}

void test_sim_brake_stops_faster_than_coasting()
{
    MotorSim coasting{MotorSimConfig()};
    MotorSim braking{MotorSimConfig()};
    uint32_t nowCoasting = 0;
    uint32_t nowBraking = 0;
    coasting.step(nowCoasting);
    braking.step(nowBraking);
    coasting.apply(driveOutput(true, 1.0f, 0.0f));
    braking.apply(driveOutput(true, 1.0f, 0.0f));
    stepSim(coasting, nowCoasting, 2.0f);
    stepSim(braking, nowBraking, 2.0f);

    coasting.apply(driveOutput(true, 0.0f, 0.0f));
    braking.apply(driveOutput(true, 0.0f, 1.0f));
    stepSim(coasting, nowCoasting, 0.2f);
    stepSim(braking, nowBraking, 0.2f);
    TEST_ASSERT_TRUE(coasting.velocity() > 200.0f);
    TEST_ASSERT_TRUE(braking.velocity() < 20.0f);
}

// ---------------------------------------------------------------- closed loop

static const float MM_PER_TICK = 3.14159265f * 220.0f / 340.0f;

struct Bench
{
    MotorSim sim{MotorSimConfig()};
    MotorAxis axis;
    uint32_t now = 0;
    float fastestReversal = 0.0f; // real speed at the moment the direction pin changed

    explicit Bench(const MotorConfig &config) : axis(sim, sim, config, MM_PER_TICK, 200000)
    {
        sim.step(now);
    }

    float realSpeed() const { return sim.velocity() * MM_PER_TICK; }

    // runs the loop like the firmware does, a new command arrives every 20 ms
    template <typename Command>
    void run(float seconds, Command command)
    {
        const int steps = static_cast<int>(lroundf(seconds / DT));
        for (int i = 0; i < steps; i++)
        {
            if (i % 4 == 0)
                command(axis.control());
            const bool forwardBefore = axis.output().forward;
            now += STEP;
            sim.step(now);
            axis.update(now, DT);
            if (axis.output().forward != forwardBefore)
                fastestReversal = fmaxf(fastestReversal, fabsf(realSpeed()));
        }
    }
};

static MotorConfig gearConfig()
{
    MotorConfig config;
    config.gains = {0.002f, 0.02f, 0.0f};
    config.pwmBraking = true;
    config.brakeMinDuty = 0.0f;
    config.brakeMaxDuty = 1.0f;
    config.standstillSpeed = 20.0f;
    return config;
}

void test_loop_reaches_and_holds_speed()
{
    Bench bench(gearConfig());
    bench.run(3.0f, [](MotorController &control) { control.setSpeed(300.0f); });
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 300.0f, bench.realSpeed());
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 300.0f, bench.axis.speed());
    TEST_ASSERT_TRUE(bench.axis.output().forward);

    // slow speed with only about 50 impulses per second
    bench.run(3.0f, [](MotorController &control) { control.setSpeed(100.0f); });
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 100.0f, bench.realSpeed());
}

void test_loop_reverses_through_standstill()
{
    Bench bench(gearConfig());
    bench.run(3.0f, [](MotorController &control) { control.setSpeed(300.0f); });
    bench.run(4.0f, [](MotorController &control) { control.setSpeed(-300.0f); });
    TEST_ASSERT_FLOAT_WITHIN(5.0f, -300.0f, bench.realSpeed());
    TEST_ASSERT_FALSE(bench.axis.output().forward);
    TEST_ASSERT_TRUE(bench.fastestReversal < 30.0f);
}

void test_loop_stops_and_holds_brake()
{
    Bench bench(gearConfig());
    bench.run(3.0f, [](MotorController &control) { control.setSpeed(300.0f); });
    bench.run(2.0f, [](MotorController &control) { control.setSpeed(0.0f); });
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, bench.realSpeed());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, bench.axis.speed());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, bench.axis.output().drive);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, bench.axis.output().brake);
}

void test_loop_stops_when_commands_end()
{
    Bench bench(gearConfig());
    bench.run(3.0f, [](MotorController &control) { control.setSpeed(300.0f); });
    bench.run(5.0f, [](MotorController &) {});
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, bench.realSpeed());
}

void test_loop_in_pwm_mode_follows_duty()
{
    Bench bench(gearConfig());
    bench.run(2.0f, [](MotorController &control) { control.setPwm(128); });
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 350.0f * 128.0f / 255.0f, bench.sim.velocity());
}

void test_loop_reports_distance_as_ticks()
{
    Bench bench(gearConfig());
    bench.run(3.0f, [](MotorController &control) { control.setSpeed(300.0f); });
    const uint32_t before = bench.axis.ticks();
    bench.run(2.0f, [](MotorController &control) { control.setSpeed(300.0f); });
    // 600 mm at 2.03 mm per impulse
    TEST_ASSERT_UINT32_WITHIN(8, 295, bench.axis.ticks() - before);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_pwm_mode_passes_duty_through);
    RUN_TEST(test_pwm_mode_backwards_and_clamped);
    RUN_TEST(test_speed_mode_drives_towards_setpoint);
    RUN_TEST(test_speed_mode_backwards_uses_unsigned_speed);
    RUN_TEST(test_too_fast_brakes_within_configured_range);
    RUN_TEST(test_too_fast_coasts_without_pwm_braking);
    RUN_TEST(test_stop_coasts_then_ramps_brake_and_holds);
    RUN_TEST(test_stop_without_pwm_braking_switches_brake_on);
    RUN_TEST(test_brake_is_not_applied_to_a_motor_at_rest);
    RUN_TEST(test_brake_releases_when_driving_again);
    RUN_TEST(test_reversing_stops_the_motor_first);
    RUN_TEST(test_missing_commands_stop_the_motor);
    RUN_TEST(test_repeated_commands_keep_the_motor_running);
    RUN_TEST(test_gains_can_be_changed);
    RUN_TEST(test_sim_reaches_speed_proportional_to_drive);
    RUN_TEST(test_sim_counts_impulses_in_both_directions);
    RUN_TEST(test_sim_impulse_time_is_never_in_the_future);
    RUN_TEST(test_sim_brake_stops_faster_than_coasting);
    RUN_TEST(test_loop_reaches_and_holds_speed);
    RUN_TEST(test_loop_reverses_through_standstill);
    RUN_TEST(test_loop_stops_and_holds_brake);
    RUN_TEST(test_loop_stops_when_commands_end);
    RUN_TEST(test_loop_in_pwm_mode_follows_duty);
    RUN_TEST(test_loop_reports_distance_as_ticks);
    return UNITY_END();
}
