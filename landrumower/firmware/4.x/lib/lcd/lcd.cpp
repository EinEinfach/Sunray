#include "lcd.h"

#include <string.h>

namespace
{

void fill(lcd::Line &line, const char *text)
{
    size_t i = 0;
    for (; (i < lcd::COLUMNS) && (text[i] != '\0'); i++)
        line.text[i] = text[i];
    for (; i < lcd::COLUMNS; i++)
        line.text[i] = ' ';
    line.text[lcd::COLUMNS] = '\0';
}

void clear(lcd::Frame &frame)
{
    for (lcd::Line &line : frame.rows)
        fill(line, "");
}

} // namespace

bool lcd::operator==(const Frame &a, const Frame &b)
{
    for (size_t row = 0; row < ROWS; row++)
    {
        if (memcmp(a.rows[row].text, b.rows[row].text, COLUMNS) != 0)
            return false;
    }
    return true;
}

LcdScreen::LcdScreen()
{
    clear(normal);
    clear(priority);
}

void LcdScreen::setLine(size_t row, const char *text)
{
    if (row < lcd::ROWS)
        fill(normal.rows[row], text);
}

void LcdScreen::showPriority(const char *first, const char *second, uint32_t nowMillis, uint32_t durationMillis)
{
    fill(priority.rows[0], first);
    fill(priority.rows[1], second);
    priorityActive = true;
    priorityStartMillis = nowMillis;
    priorityDurationMillis = durationMillis;
}

lcd::Frame LcdScreen::frame(uint32_t nowMillis)
{
    if (priorityActive && (nowMillis - priorityStartMillis >= priorityDurationMillis))
        priorityActive = false;
    return priorityActive ? priority : normal;
}
