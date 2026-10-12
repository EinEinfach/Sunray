#include <pid.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

static const float TOLERANCE = 1e-4f;

void test_proportional_only()
{
    Pid pid({2.0f, 0.0f, 0.0f}, -100.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 10.0f, pid.update(5.0f, 0.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -6.0f, pid.update(0.0f, 3.0f, 0.01f));
}

void test_integral_accumulates_over_time()
{
    Pid pid({0.0f, 1.0f, 0.0f}, -100.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 1.0f, pid.update(2.0f, 0.0f, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 2.0f, pid.update(2.0f, 0.0f, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 2.0f, pid.update(2.0f, 2.0f, 0.5f));
}

void test_derivative_acts_on_measurement()
{
    Pid pid({0.0f, 0.0f, 1.0f}, -100.0f, 100.0f);
    // first call has no history
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, pid.update(0.0f, 0.0f, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -10.0f, pid.update(0.0f, 1.0f, 0.1f));
}

void test_setpoint_step_causes_no_derivative_kick()
{
    Pid pid({0.0f, 0.0f, 1.0f}, -100.0f, 100.0f);
    pid.update(0.0f, 1.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, pid.update(50.0f, 1.0f, 0.1f));
}

void test_output_is_clamped()
{
    Pid pid({10.0f, 0.0f, 0.0f}, -5.0f, 20.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 20.0f, pid.update(100.0f, 0.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -5.0f, pid.update(-100.0f, 0.0f, 0.01f));
}

void test_integral_does_not_wind_up_while_saturated()
{
    Pid pid({0.0f, 1.0f, 0.0f}, -10.0f, 10.0f);
    for (int i = 0; i < 50; i++)
        TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 10.0f, pid.update(100.0f, 0.0f, 1.0f));
    // a wound-up integral would keep the output at the limit for many more steps
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -1.0f, pid.update(0.0f, 1.0f, 1.0f));
}

void test_reset_clears_state()
{
    Pid pid({0.0f, 1.0f, 1.0f}, -100.0f, 100.0f);
    pid.update(5.0f, 0.0f, 1.0f);
    pid.update(5.0f, 1.0f, 1.0f);
    pid.reset();
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, pid.update(0.0f, 7.0f, 0.0f));
    // error -7 integrates to -7, no derivative because the history is gone
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -7.0f, pid.update(0.0f, 7.0f, 1.0f));
}

void test_non_positive_dt_keeps_last_output()
{
    Pid pid({1.0f, 0.0f, 0.0f}, -100.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 3.0f, pid.update(3.0f, 0.0f, 0.01f));
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 3.0f, pid.update(50.0f, 0.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 3.0f, pid.update(50.0f, 0.0f, -1.0f));
}

void test_changed_gains_apply_to_next_update()
{
    Pid pid({1.0f, 0.0f, 0.0f}, -100.0f, 100.0f);
    pid.setGains({4.0f, 0.0f, 0.0f});
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 8.0f, pid.update(2.0f, 0.0f, 0.01f));
    pid.setOutputLimits(-1.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 1.0f, pid.update(2.0f, 0.0f, 0.01f));
}

void test_closed_loop_reaches_setpoint()
{
    // first-order plant: speed follows the output with a time constant of 100 ms
    const float dt = 0.001f;
    const float timeConstant = 0.1f;
    Pid pid({0.5f, 5.0f, 0.0f}, 0.0f, 255.0f);
    float speed = 0.0f;
    for (int i = 0; i < 5000; i++)
    {
        const float drive = pid.update(100.0f, speed, dt);
        speed += (drive - speed) * dt / timeConstant;
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, speed);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_proportional_only);
    RUN_TEST(test_integral_accumulates_over_time);
    RUN_TEST(test_derivative_acts_on_measurement);
    RUN_TEST(test_setpoint_step_causes_no_derivative_kick);
    RUN_TEST(test_output_is_clamped);
    RUN_TEST(test_integral_does_not_wind_up_while_saturated);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_non_positive_dt_keeps_last_output);
    RUN_TEST(test_changed_gains_apply_to_next_update);
    RUN_TEST(test_closed_loop_reaches_setpoint);
    return UNITY_END();
}
