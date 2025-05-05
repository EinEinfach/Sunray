#pragma once
#include <Arduino.h>
#include <vector>

class Buzzer {
public:
    Buzzer(uint pin);
    void setup();
    void run(const String& mainUnitState);
    void stopPlaying();
    void playInfo(uint16_t loudness);
    void playImuCalibration(uint16_t loudness);
    void playWarning(uint16_t loudness);
    void playShutdown(uint16_t loudness);
    void checkPlayPattern(const String& mainUnitState);

private:
    uint8_t pin;
    String mainUnitState = "";
    bool sound = false;
    std::vector<int> playPattern;
    uint32_t currentTime = 0;
    uint16_t loudness = 0;
    uint32_t nextRunTime = 0;
    uint32_t runFrequency = 20; // in ms
};