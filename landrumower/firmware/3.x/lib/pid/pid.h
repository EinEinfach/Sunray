#include <Arduino.h>

class PID
{
    public:
        PID();
        void setup(float kP, float kI, float kD, int outputMin, int outputMax, int spampleTime);
        void reset();
        int coumpute(int currentSetPoint, int currentValue);
    private:
        float kP, kI, kD;
        int outputMin, outputMax;
        int sampleTime;
        int lastComputeTime;
        int output;
        int lastError;
        int integral;
};