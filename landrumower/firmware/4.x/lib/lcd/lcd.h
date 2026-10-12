// Content of the 16x2 LCD: two normal lines and a priority message that covers them for a
// limited time (for example "shutdown").
//
// LcdScreen only decides what is shown. Writing to the display is done by the caller.

#ifndef LCD_H
#define LCD_H

#include <stddef.h>
#include <stdint.h>

namespace lcd
{
constexpr size_t COLUMNS = 16;
constexpr size_t ROWS = 2;

// one display line, always COLUMNS characters, padded with spaces
struct Line
{
    char text[COLUMNS + 1];
};

struct Frame
{
    Line rows[ROWS];
};
} // namespace lcd

class LcdScreen
{
public:
    LcdScreen();

    void setLine(size_t row, const char *text);
    void showPriority(const char *first, const char *second, uint32_t nowMillis, uint32_t durationMillis);

    // what the display has to show now
    lcd::Frame frame(uint32_t nowMillis);

private:
    lcd::Frame normal;
    lcd::Frame priority;
    bool priorityActive = false;
    uint32_t priorityStartMillis = 0;
    uint32_t priorityDurationMillis = 0;
};

namespace lcd
{
bool operator==(const Frame &a, const Frame &b);
inline bool operator!=(const Frame &a, const Frame &b) { return !(a == b); }
} // namespace lcd

#endif // LCD_H
