#include <buzzer.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

// total sounding time between two points in time, sampled every millisecond
static uint32_t soundMillis(BuzzerPlayer &player, uint32_t from, uint32_t to)
{
    uint32_t total = 0;
    for (uint32_t now = from; now != to; now++)
    {
        if (player.update(now))
            total++;
    }
    return total;
}

void test_silent_without_pattern()
{
    BuzzerPlayer player;
    TEST_ASSERT_FALSE(player.update(0));
    TEST_ASSERT_FALSE(player.update(100000));
}

void test_info_is_one_second_of_sound()
{
    BuzzerPlayer player;
    player.play(buzzer::INFO, 5000);
    TEST_ASSERT_TRUE(player.update(5000));
    TEST_ASSERT_TRUE(player.update(5999));
    TEST_ASSERT_FALSE(player.update(6000));
    TEST_ASSERT_FALSE(player.update(60000));
}

void test_warning_is_three_beeps()
{
    BuzzerPlayer player;
    player.play(buzzer::WARNING, 0);
    TEST_ASSERT_TRUE(player.update(0));
    TEST_ASSERT_TRUE(player.update(499));
    TEST_ASSERT_FALSE(player.update(500));
    TEST_ASSERT_FALSE(player.update(1499));
    TEST_ASSERT_TRUE(player.update(1500));
    TEST_ASSERT_FALSE(player.update(2000));
    TEST_ASSERT_TRUE(player.update(3000));
    TEST_ASSERT_TRUE(player.update(3499));
    TEST_ASSERT_FALSE(player.update(3500));
}

void test_pattern_durations()
{
    BuzzerPlayer player;
    player.play(buzzer::IMU_CALIBRATION, 0);
    TEST_ASSERT_EQUAL_UINT32(12 * 100, soundMillis(player, 0, 30000));
    player.play(buzzer::SHUTDOWN, 0);
    TEST_ASSERT_EQUAL_UINT32(16 * 100, soundMillis(player, 0, 60000));
}

void test_late_update_skips_ahead()
{
    BuzzerPlayer player;
    player.play(buzzer::WARNING, 0);
    // called only every 700 ms, as after a slow LCD write
    TEST_ASSERT_TRUE(player.update(0));
    TEST_ASSERT_FALSE(player.update(700));
    TEST_ASSERT_FALSE(player.update(1400));
    TEST_ASSERT_FALSE(player.update(2100));
    TEST_ASSERT_FALSE(player.update(2800));
    TEST_ASSERT_FALSE(player.update(3500));
}

void test_time_counter_may_wrap()
{
    BuzzerPlayer player;
    const uint32_t start = 0xFFFFFFFFu - 300;
    player.play(buzzer::INFO, start);
    TEST_ASSERT_TRUE(player.update(start + 999));
    TEST_ASSERT_FALSE(player.update(start + 1000));
}

void test_state_change_plays_its_pattern_once()
{
    BuzzerPlayer player;
    player.setState(MowerState::Boot, 0);
    TEST_ASSERT_EQUAL_UINT32(1000, soundMillis(player, 0, 5000));

    // the same state again does not restart the pattern
    player.setState(MowerState::Boot, 5000);
    TEST_ASSERT_EQUAL_UINT32(0, soundMillis(player, 5000, 8000));

    player.setState(MowerState::Mow, 8000);
    TEST_ASSERT_EQUAL_UINT32(1500, soundMillis(player, 8000, 20000));
}

void test_new_state_interrupts_running_pattern()
{
    BuzzerPlayer player;
    player.setState(MowerState::ImuCalibration, 0);
    soundMillis(player, 0, 2000);
    player.setState(MowerState::Idle, 2000);
    TEST_ASSERT_EQUAL_UINT32(1000, soundMillis(player, 2000, 30000));
}

void test_states_without_sound()
{
    BuzzerPlayer player;
    player.setState(MowerState::KidnapWait, 0);
    TEST_ASSERT_EQUAL_UINT32(0, soundMillis(player, 0, 5000));
    player.setState(MowerState::Unknown, 5000);
    TEST_ASSERT_EQUAL_UINT32(0, soundMillis(player, 5000, 10000));
}

void test_shutdown_is_not_interrupted()
{
    BuzzerPlayer player;
    player.setState(MowerState::Shutdown, 0);
    soundMillis(player, 0, 1000);
    player.setState(MowerState::Idle, 1000);
    // 200 ms of the shutdown pattern sounded in the first second, the rest still follows
    TEST_ASSERT_EQUAL_UINT32(1400, soundMillis(player, 1000, 60000));
}

void test_pattern_of_each_state()
{
    TEST_ASSERT_EQUAL_PTR(&buzzer::INFO, &buzzer::patternFor(MowerState::Boot));
    TEST_ASSERT_EQUAL_PTR(&buzzer::INFO, &buzzer::patternFor(MowerState::Idle));
    TEST_ASSERT_EQUAL_PTR(&buzzer::INFO, &buzzer::patternFor(MowerState::Charge));
    TEST_ASSERT_EQUAL_PTR(&buzzer::INFO, &buzzer::patternFor(MowerState::Dock));
    TEST_ASSERT_EQUAL_PTR(&buzzer::INFO, &buzzer::patternFor(MowerState::GpsRecovery));
    TEST_ASSERT_EQUAL_PTR(&buzzer::INFO, &buzzer::patternFor(MowerState::GpsWaitFix));
    TEST_ASSERT_EQUAL_PTR(&buzzer::INFO, &buzzer::patternFor(MowerState::GpsWaitFloat));
    TEST_ASSERT_EQUAL_PTR(&buzzer::WARNING, &buzzer::patternFor(MowerState::Mow));
    TEST_ASSERT_EQUAL_PTR(&buzzer::WARNING, &buzzer::patternFor(MowerState::Error));
    TEST_ASSERT_EQUAL_PTR(&buzzer::WARNING, &buzzer::patternFor(MowerState::EscapeForward));
    TEST_ASSERT_EQUAL_PTR(&buzzer::WARNING, &buzzer::patternFor(MowerState::EscapeReverse));
    TEST_ASSERT_EQUAL_PTR(&buzzer::IMU_CALIBRATION, &buzzer::patternFor(MowerState::ImuCalibration));
    TEST_ASSERT_EQUAL_PTR(&buzzer::SHUTDOWN, &buzzer::patternFor(MowerState::Shutdown));
    TEST_ASSERT_EQUAL_PTR(&buzzer::SILENCE, &buzzer::patternFor(MowerState::KidnapWait));
    TEST_ASSERT_EQUAL_PTR(&buzzer::SILENCE, &buzzer::patternFor(MowerState::Unknown));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_silent_without_pattern);
    RUN_TEST(test_info_is_one_second_of_sound);
    RUN_TEST(test_warning_is_three_beeps);
    RUN_TEST(test_pattern_durations);
    RUN_TEST(test_late_update_skips_ahead);
    RUN_TEST(test_time_counter_may_wrap);
    RUN_TEST(test_state_change_plays_its_pattern_once);
    RUN_TEST(test_new_state_interrupts_running_pattern);
    RUN_TEST(test_states_without_sound);
    RUN_TEST(test_shutdown_is_not_interrupted);
    RUN_TEST(test_pattern_of_each_state);
    return UNITY_END();
}
