#include <mower_state.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_sunray_numbers_map_to_states()
{
    // numbering from sunray/robot.cpp
    TEST_ASSERT_TRUE(mowerStateFromSunray(0) == MowerState::Idle);
    TEST_ASSERT_TRUE(mowerStateFromSunray(1) == MowerState::Mow);
    TEST_ASSERT_TRUE(mowerStateFromSunray(2) == MowerState::Charge);
    TEST_ASSERT_TRUE(mowerStateFromSunray(3) == MowerState::Error);
    TEST_ASSERT_TRUE(mowerStateFromSunray(4) == MowerState::Dock);
    TEST_ASSERT_TRUE(mowerStateFromSunray(5) == MowerState::EscapeForward);
    TEST_ASSERT_TRUE(mowerStateFromSunray(6) == MowerState::EscapeReverse);
    TEST_ASSERT_TRUE(mowerStateFromSunray(7) == MowerState::GpsRecovery);
    TEST_ASSERT_TRUE(mowerStateFromSunray(8) == MowerState::GpsWaitFix);
    TEST_ASSERT_TRUE(mowerStateFromSunray(9) == MowerState::GpsWaitFloat);
    TEST_ASSERT_TRUE(mowerStateFromSunray(10) == MowerState::ImuCalibration);
    TEST_ASSERT_TRUE(mowerStateFromSunray(11) == MowerState::KidnapWait);
}

void test_other_numbers_are_unknown()
{
    TEST_ASSERT_TRUE(mowerStateFromSunray(-1) == MowerState::Unknown);
    TEST_ASSERT_TRUE(mowerStateFromSunray(12) == MowerState::Unknown);
    TEST_ASSERT_TRUE(mowerStateFromSunray(1000) == MowerState::Unknown);
}

void test_texts()
{
    TEST_ASSERT_EQUAL_STRING("boot", mowerStateText(MowerState::Boot));
    TEST_ASSERT_EQUAL_STRING("mow", mowerStateText(MowerState::Mow));
    TEST_ASSERT_EQUAL_STRING("gps wait float", mowerStateText(MowerState::GpsWaitFloat));
    TEST_ASSERT_EQUAL_STRING("shutdown", mowerStateText(MowerState::Shutdown));
    TEST_ASSERT_EQUAL_STRING("unknown", mowerStateText(MowerState::Unknown));
}

void test_every_text_fits_the_display()
{
    for (int number = -1; number <= 12; number++)
    {
        const char *text = mowerStateText(mowerStateFromSunray(number));
        size_t length = 0;
        while (text[length] != '\0')
            length++;
        TEST_ASSERT_TRUE(length > 0);
        TEST_ASSERT_TRUE(length <= 16);
    }
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_sunray_numbers_map_to_states);
    RUN_TEST(test_other_numbers_are_unknown);
    RUN_TEST(test_texts);
    RUN_TEST(test_every_text_fits_the_display);
    return UNITY_END();
}
