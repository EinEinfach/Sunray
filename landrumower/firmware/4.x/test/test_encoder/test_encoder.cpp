#include <encoder.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

static const uint32_t TIMEOUT = 200000;
static const uint32_t STEP = 5000;

// encoder turning at a constant rate since startMicros
static EncoderSample turning(uint32_t nowMicros, uint32_t startMicros, uint32_t periodMicros)
{
    EncoderSample sample;
    sample.ticks = (nowMicros - startMicros) / periodMicros;
    sample.lastEdgeMicros = startMicros + sample.ticks * periodMicros;
    return sample;
}

void test_constant_rate_is_measured_exactly()
{
    SpeedEstimator estimator(TIMEOUT);
    float speed = 0.0f;
    // 150 impulses per second, the control loop samples faster than impulses arrive
    for (uint32_t now = 0; now < 500000; now += STEP)
        speed = estimator.update(turning(now, 0, 6667), now);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 149.99f, speed);
}

void test_several_impulses_between_samples()
{
    SpeedEstimator estimator(TIMEOUT);
    float speed = 0.0f;
    for (uint32_t now = 0; now < 100000; now += STEP)
        speed = estimator.update(turning(now, 0, 400), now);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2500.0f, speed);
}

void test_speed_needs_two_impulses_after_standstill()
{
    SpeedEstimator estimator(TIMEOUT);
    EncoderSample sample;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, estimator.update(sample, 0));

    sample.ticks = 1;
    sample.lastEdgeMicros = 10000;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, estimator.update(sample, 10000));

    sample.ticks = 2;
    sample.lastEdgeMicros = 20000;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, estimator.update(sample, 20000));
}

void test_speed_decays_and_reaches_zero_without_impulses()
{
    SpeedEstimator estimator(TIMEOUT);
    EncoderSample sample;
    for (uint32_t now = 0; now <= 100000; now += STEP)
        sample = turning(now, 0, 10000), estimator.update(sample, now);

    // last impulse at 100 ms. Within one period the estimate is kept.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, estimator.update(sample, 105000));
    // after 50 ms without impulse the motor cannot be faster than 20 per second
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, estimator.update(sample, 150000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, estimator.update(sample, 200000));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, estimator.update(sample, 100000 + TIMEOUT));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, estimator.update(sample, 100000 + TIMEOUT + STEP));
}

void test_restart_after_timeout_ignores_the_pause()
{
    SpeedEstimator estimator(TIMEOUT);
    EncoderSample sample;
    for (uint32_t now = 0; now <= 100000; now += STEP)
        sample = turning(now, 0, 10000), estimator.update(sample, now);
    estimator.update(sample, 1000000);

    // first impulse after one second of standstill must not be read as a very slow speed
    sample.ticks += 1;
    sample.lastEdgeMicros = 1100000;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, estimator.update(sample, 1100000));
    sample.ticks += 1;
    sample.lastEdgeMicros = 1105000;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, estimator.update(sample, 1105000));
}

void test_time_and_tick_counters_may_wrap()
{
    SpeedEstimator estimator(TIMEOUT);
    const uint32_t start = 0xFFFFFFFFu - 50000;
    float speed = 0.0f;
    for (uint32_t elapsed = 0; elapsed < 200000; elapsed += STEP)
    {
        const uint32_t now = start + elapsed;
        EncoderSample sample = turning(now, start, 10000);
        sample.ticks += 0xFFFFFFFFu - 3; // tick counter wraps as well
        speed = estimator.update(sample, now);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, speed);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_constant_rate_is_measured_exactly);
    RUN_TEST(test_several_impulses_between_samples);
    RUN_TEST(test_speed_needs_two_impulses_after_standstill);
    RUN_TEST(test_speed_decays_and_reaches_zero_without_impulses);
    RUN_TEST(test_restart_after_timeout_ignores_the_pause);
    RUN_TEST(test_time_and_tick_counters_may_wrap);
    return UNITY_END();
}
