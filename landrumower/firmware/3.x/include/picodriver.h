#include <Arduino.h>
#include <LCD_I2C.h>
#include <hardware/uart.h>
#include "buzzer.h"
#include "motor.h"
#include "battery.h"
#include "digsensor.h"

class PicoDriver
{
public:
    PicoDriver();
    void setup();
    void run();
    Buzzer buzzer;

private:
    String mainUnitState;
    String cmd;
    String cmdResponse;
    String printedMessage;
    LCD_I2C lcd;
    uint16_t motorTimeout;
    Motor motorRight;
    Motor motorLeft;
    Motor motorMow;
    Battery battery;
    DigSensor bumperX;
    DigSensor bumperY;
    DigSensor lift;
    DigSensor stop;
    void cmdAnswer(String s);
    void cmdVersion();
    void cmdMotor();
    void processCmd(bool checkCrc);
    void processConsole();
    void printLcd(String message);
};