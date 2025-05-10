#include <Arduino.h>
#include <INA226.h>

class Battery
{
public:
    Battery(uint8_t pinPowerSwitch);
    uint8_t pinPowerSwitch;
    bool sensorConnected;
    bool chgConnected;
    bool requestShutdown;
    int shutdownRequestTime;
    float voltage;
    float chgVoltage;
    float chgCurrent;
    float temperature;
    void setup();
    void run();
private:
    int nextRunTime;
    int nextRunTimeHighFreq;
    INA226 ina;
    void connectSensor();
    void readVoltage();
    void readCurrent();
    void checkCharger();
    void keepPowerOn();
};