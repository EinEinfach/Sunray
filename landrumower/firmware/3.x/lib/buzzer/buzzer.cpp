#include <Arduino.h>
#include "config.h"
#include "buzzer.h"

Buzzer::Buzzer(uint8_t pin)
{
    this->pin = pin;
}

void Buzzer::setup()
{
    pinMode(pin, OUTPUT);
}

void Buzzer::stopPlaying()
{
    sound = false;
    loudness = 0;
    playPattern.clear();
    currentTime = 0;
}

void Buzzer::playInfo(int loudness)
{
    stopPlaying();
    this->loudness = loudness;
    playPattern = {1000};
}

void Buzzer::playImuCalibration(int loudness)
{
    stopPlaying();
    this->loudness = loudness;
    playPattern = {
        100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000,
        100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000,
        100, -1000, 100};
}

void Buzzer::playWarning(int loudness)
{
    stopPlaying();
    this->loudness = loudness;
    playPattern = {500, -1000, 500, -1000, 500};
}

void Buzzer::playShutdown(int loudness)
{
    stopPlaying();
    this->loudness = loudness;
    playPattern = {
        100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20,
        100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000,
        100, -20, 100, -3000, 100, -20, 100, -3000};
}

void Buzzer::checkPlayPattern(const String &state)
{
    mainUnitState = state;

    if (state == "boot" || state == "idle" || state == "charge" || state == "dock" ||
        state == "gps recovery" || state == "gps wait fix" || state == "gps wait float")
    {
        playInfo(BUZZER_LOUDNESS);
    }
    else if (state == "imu calibration")
    {
        playImuCalibration(BUZZER_LOUDNESS);
    }
    else if (state == "mow" || state == "error" || state == "escape forward" || state == "escape reverse")
    {
        playWarning(BUZZER_LOUDNESS);
    }
}

void Buzzer::run(const String &state)
{
    int now = millis();

    if ((now - nextRunTime) >= 0)
    {
        nextRunTime = now + runFrequency;

        if (mainUnitState != state && mainUnitState != "shutdown")
        {
            checkPlayPattern(state);
        }

        if ((now - currentTime) >= 0)
        {
            if (playPattern.empty())
            {
                stopPlaying();
            }
            else
            {
                int duration = playPattern.front();
                playPattern.erase(playPattern.begin());
                sound = duration > 0;
                currentTime = now + abs(duration);
            }
        }

        if (sound)
        {
            //digitalWrite(pin, HIGH);
            analogWrite(pin, loudness);
        }
        else
        {
            //digitalWrite(pin, LOW);
            analogWrite(pin, 0);
        }
    }
}