#include <Arduino.h>
#include <INA226.h>

class Battery
{
public:
    Battery(uint8_t pinPowerSwitch);
    uint8_t pinPowerSwitch;
    bool sensorConnected;
    float voltage;
    float chgVoltage;
    float chgCurrent;
    float temperature;
    void setup();
    void run();
private:
    uint32_t nextRunTime;
    uint32_t nextRunTimeHighFreq;
    INA226 ina;
    void connectSensor();
    void readVoltage();
    void readCurrent();
};