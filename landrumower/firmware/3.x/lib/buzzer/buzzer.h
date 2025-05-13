#pragma once
#include <Arduino.h>
#include <vector>

class Buzzer {
public:
    Buzzer(uint8_t pin);
    void setup();
    void run(const String& mainUnitState);
    void stopPlaying();
    void playInfo(int loudness);
    void playImuCalibration(int loudness);
    void playWarning(int loudness);
    void playShutdown(int loudness);
    void checkPlayPattern(const String& mainUnitState);

private:
    uint8_t pin;
    String mainUnitState = "";
    bool sound = false;
    std::vector<int> playPattern;
    int currentTime = 0;
    int loudness = 0;
    int nextRunTime = 0;
    int runFrequency = 20; // in ms
};