#include <lcd.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_empty_screen_is_all_spaces()
{
    LcdScreen screen;
    const lcd::Frame frame = screen.frame(0);
    TEST_ASSERT_EQUAL_STRING("                ", frame.rows[0].text);
    TEST_ASSERT_EQUAL_STRING("                ", frame.rows[1].text);
}

void test_short_text_is_padded()
{
    LcdScreen screen;
    screen.setLine(0, "mow");
    screen.setLine(1, "25.2V/1.5A");
    const lcd::Frame frame = screen.frame(0);
    TEST_ASSERT_EQUAL_STRING("mow             ", frame.rows[0].text);
    TEST_ASSERT_EQUAL_STRING("25.2V/1.5A      ", frame.rows[1].text);
}

void test_long_text_is_cut()
{
    LcdScreen screen;
    screen.setLine(0, "0123456789abcdefXYZ");
    TEST_ASSERT_EQUAL_STRING("0123456789abcdef", screen.frame(0).rows[0].text);
}

void test_shorter_text_removes_the_old_one()
{
    LcdScreen screen;
    screen.setLine(0, "imu calibration");
    screen.setLine(0, "idle");
    TEST_ASSERT_EQUAL_STRING("idle            ", screen.frame(0).rows[0].text);
}

void test_invalid_row_is_ignored()
{
    LcdScreen screen;
    screen.setLine(2, "nothing");
    TEST_ASSERT_EQUAL_STRING("                ", screen.frame(0).rows[0].text);
    TEST_ASSERT_EQUAL_STRING("                ", screen.frame(0).rows[1].text);
}

void test_priority_message_covers_lines_for_its_duration()
{
    LcdScreen screen;
    screen.setLine(0, "mow");
    screen.showPriority("motor drivers", "recovery", 1000, 5000);
    TEST_ASSERT_EQUAL_STRING("motor drivers   ", screen.frame(1000).rows[0].text);
    TEST_ASSERT_EQUAL_STRING("recovery        ", screen.frame(5999).rows[1].text);

    // lines changed meanwhile appear afterwards
    screen.setLine(0, "idle");
    TEST_ASSERT_EQUAL_STRING("motor drivers   ", screen.frame(5999).rows[0].text);
    TEST_ASSERT_EQUAL_STRING("idle            ", screen.frame(6000).rows[0].text);
    TEST_ASSERT_EQUAL_STRING("                ", screen.frame(6000).rows[1].text);
}

void test_priority_duration_with_wrapping_time()
{
    LcdScreen screen;
    screen.setLine(0, "idle");
    const uint32_t start = 0xFFFFFFFFu - 1000;
    screen.showPriority("shutdown", "", start, 5000);
    TEST_ASSERT_EQUAL_STRING("shutdown        ", screen.frame(start + 4999).rows[0].text);
    TEST_ASSERT_EQUAL_STRING("idle            ", screen.frame(start + 5000).rows[0].text);
}

void test_frames_compare_by_content()
{
    LcdScreen screen;
    screen.setLine(0, "mow");
    const lcd::Frame first = screen.frame(0);
    TEST_ASSERT_TRUE(first == screen.frame(100));
    screen.setLine(0, "mow");
    TEST_ASSERT_TRUE(first == screen.frame(200));
    screen.setLine(1, "x");
    TEST_ASSERT_TRUE(first != screen.frame(300));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_empty_screen_is_all_spaces);
    RUN_TEST(test_short_text_is_padded);
    RUN_TEST(test_long_text_is_cut);
    RUN_TEST(test_shorter_text_removes_the_old_one);
    RUN_TEST(test_invalid_row_is_ignored);
    RUN_TEST(test_priority_message_covers_lines_for_its_duration);
    RUN_TEST(test_priority_duration_with_wrapping_time);
    RUN_TEST(test_frames_compare_by_content);
    return UNITY_END();
}
