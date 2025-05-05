#include <Arduino.h>
#include <hardware/uart.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include "config.h"
#include "picodriver.h"

PicoDriver::PicoDriver()
    : buzzer(PIN_BUZZER),
      lcd(LCD_I2C(LCDADRESS, LCD_NUM_COLUMNS, LCD_NUM_ROWS)),
      motorRight(RIGHT_IMP, RIGHT_PWM, RIGHT_DIR, RIGHT_PWM),
      motorLeft(LEFT_IMP, LEFT_PWM, LEFT_DIR, LEFT_PWM),
      motorMow(MOW_IMP, MOW_PWM, MOW_DIR, MOW_PWM),
      battery(POWER_SWITCH),
      bumperX(BUMPER_X, false),
      bumperY(BUMPER_Y, false),
      lift(LIFT, false),
      stop(STOP, false),
      rain(RAIN, true, RAINSENSOR_THRESHOLD)
{
    // Initialize
    nextInfoTime = 0;
    lps = 0;
    lps2 = 0;
    mainUnitState = "boot";
    cmd = "";
    cmdResponse = "";
    motorTimeout = 0;
    buzzer.stopPlaying();
}

void PicoDriver::setup()
{
    // configure uart
    UART.setTX(UART0_TX);
    UART.setRX(UART0_RX);
    UART.begin(UART_BAUDRATE);
    USB.begin(USB_BAUDRATE);
    // configure i2c0 (INAs)
    // configuration of i2c1 is done by LCD_I2C library
    Wire.setSDA(I2C0_SDA);
    Wire.setSCL(I2C0_SCL);
    Wire.setClock(I2C0_CLOCK);
    Wire.begin();

    // switch power on
    battery.setup();

    printedMessage = "";
    if (LCD)
    {
        lcd.begin();
        lcd.backlight();
    }
    buzzer.setup();
    rain.setup();
    bumperX.setup();
    bumperY.setup();
    lift.setup();
    motorLeft.setup();
    motorRight.setup();
    motorMow.setup();
    USB.print(VER);
    USB.println(VERNR);
}

void PicoDriver::run()
{
    battery.run();
    buzzer.run(mainUnitState);
    rain.run();
    bumperX.run();
    bumperY.run();
    lift.run();
    processConsole();
    printInfo();
    lps++;
}

void PicoDriver::cmdAnswer(String s)
{
    byte crc = 0;
    for (int i = 0; i < s.length(); i++)
        crc += s[i];
    s += F(",0x");
    if (crc <= 0xF)
        s += F("0");
    s += String(crc, HEX);
    s += F("\r\n");
    // CONSOLE.print(s);
    cmdResponse = s;
}

void PicoDriver::cmdVersion()
{
    String s = F("V,");
    s += F(VER);
    cmdAnswer(s);
}

void PicoDriver::cmdMotor()
{
    if (cmd.length() < 6)
        return;
    int counter = 0;
    int lastCommaIdx = 0;
    int leftPwm = 0;
    int rightPwm = 0;
    int mowPwm = 0;
    int leftSpeed = 0;
    int rightSpeed = 0;
    for (int idx = 0; idx < cmd.length(); idx++)
    {
        char ch = cmd[idx];
        if ((ch == ',') || (idx == cmd.length() - 1))
        {
            int intValue = cmd.substring(lastCommaIdx + 1, ch == ',' ? idx : idx + 1).toInt();
            if (counter == 1)
            {
                rightPwm = intValue;
            }
            else if (counter == 2)
            {
                leftPwm = intValue;
            }
            else if (counter == 3)
            {
                mowPwm = intValue;
            }
            else if (counter == 4)
            {
                rightSpeed = intValue;
            }
            else if (counter == 5)
            {
                leftSpeed = intValue;
            }
            counter++;
            lastCommaIdx = idx;
        }
    }
#ifdef DEBUG
    USB.print("leftPwm=");
    USB.print(leftPwm);
    USB.print(" rightPwm=");
    USB.print(rightPwm);
    USB.print(" mowPwm=");
    USB.print(mowPwm);
    USB.print(" leftSpeed=");
    USB.print(leftSpeed);
    USB.print(" rightSpeed=");
    USB.println(rightSpeed);
#endif
    motorTimeout = millis() + 3000;
    String s = F("M");
    s += ",";
    s += int(motorLeft.odomTicks);
    s += ",";
    s += int(motorRight.odomTicks);
    s += ",";
    s += int(motorMow.odomTicks);
    s += ",";
    s += battery.chgVoltage;
    s += ",";
    s += int(bumperX.triggered || bumperY.triggered);
    s += ",";
    s += int(lift.triggered);
    s += ",";
    s += int(stop.triggered);
    cmdAnswer(s);
}

void PicoDriver::cmdSummary()
{
    int counter = 0;
    int lastCommaIdx = 0;
    int state = -1;
    if (cmd.length() > 5)
    {
        for (int idx = 0; idx < cmd.length(); idx++)
        {
            char ch = cmd[idx];
            if ((ch == ',') || (idx == cmd.length() - 1))
            {
                int intValue = cmd.substring(lastCommaIdx + 1, ch == ',' ? idx : idx + 1).toInt();
                if (counter == 1)
                {
                    state = intValue;
                }
            }
        }
    }
    String s = F("S,");
    s += battery.voltage;
    s += ",";
    s += battery.chgVoltage;
    s += ",";
    s += battery.chgCurrent;
    s += ",";
    s += int(lift.triggered);
    s += ",";
    s += int(bumperX.triggered || bumperY.triggered);
    s += ",";
    s += int(rain.triggered);
    s += ",";
    s += int(motorLeft.overload || motorRight.overload || motorMow.overload);
    s += ",";
    s += motorMow.electricalCurrent;
    s += ",";
    s += motorLeft.electricalCurrent;
    s += ",";
    s += motorRight.electricalCurrent;
    s += ",";
    s += battery.temperature;
    cmdAnswer(s);
}

void PicoDriver::processCmd(bool checkCrc)
{
    cmdResponse = "";
    if (cmd.length() < 4)
        return;
    byte expectedCrc = 0;
    int idx = cmd.lastIndexOf(',');
    if (idx < 1)
    {
        if (checkCrc)
        {
#ifdef DEBUG
            USB.println("CRC ERROR, message to short");
#endif
            return;
        }
    }
    else
    {
        for (int i = 0; i < idx; i++)
            expectedCrc += cmd[i];
        String s = cmd.substring(idx + 1, idx + 5);
        int crc = strtol(s.c_str(), NULL, 16);
        if (expectedCrc != crc)
        {
            if (checkCrc)
            {
#ifdef DEBUG
                USB.print("CRC ERROR, received CRC: ");
                USB.print(crc, HEX);
                USB.print(", expected CRC: ");
                USB.print(expectedCrc, HEX);
                USB.println();
#endif
                return;
            }
        }
        else
        {
            // remove CRC
            cmd = cmd.substring(0, idx);
        }
    }
    if (cmd[0] != 'A')
        return;
    if (cmd[1] != 'T')
        return;
    if (cmd[2] != '+')
        return;
    if (cmd[3] == 'V')
        cmdVersion();
    if (cmd[3] == 'M')
        cmdMotor();
    // if (cmd[3] == 'E')
    //     //cmdMotorControlTest
    // if (cmd[3] == 'R')
    //     //cmdResetMotorFaults()
    if (cmd[3] == 'S')
        cmdSummary();
    // if (cmd[3] == 'Y')
    // {
    //     if (cmd.length() <= 4)
    //     {
    //         //cmdTriggerWatchdog(); // for developers
    //     }
    //     else
    //     {
    //         if (cmd[4] == '3')
    //         //cmdShutdown();
    //     }
    // }
}

void PicoDriver::processConsole()
{
    char ch;
    if (UART.available())
    {
        unsigned long timeout = millis() + 10;
        while ((UART.available()) && (millis() < timeout))
        {
            ch = UART.read();
            if ((ch == '\r') || (ch == '\n'))
            {
#ifdef DEBUG
                USB.println(cmd);
#endif
                processCmd(true);
                UART.print(cmdResponse);
                cmd = "";
            }
            else if (cmd.length() < 500)
            {
                cmd += ch;
            }
        }
    }
    if (USB.available())
    {
        unsigned long timeout = millis() + 10;
        // battery.resetIdle();
        while ((USB.available()) && (millis() < timeout))
        {
            ch = USB.read();
            if ((ch == '\r') || (ch == '\n'))
            {
#ifdef DEBUG
                USB.println(cmd);
#endif
                processCmd(false);
                USB.print(cmdResponse);
                cmd = "";
            }
            else if (cmd.length() < 500)
            {
                cmd += ch;
            }
        }
    }
}

void PicoDriver::printLcd(String message)
{
    if (printedMessage != message)
    {
        lcd.print(message);
        printedMessage = message;
    }
}

void PicoDriver::printInfo()
{
    if (INFO)
    {
        int now = millis();
        if ((nextInfoTime - now) < 0) {
            USB.printf("tim=%d", now);
            USB.printf(" lps=%d/%ds", lps, int(INFOTIME/1000));
            USB.printf(" bat=%fV", battery.voltage);
            USB.printf(" chg=%fA", battery.chgCurrent);
            USB.printf(" imp=%d,%d,%d", motorLeft.odomTicks, motorRight.odomTicks, motorMow.odomTicks);
            USB.printf(" curr=0,0,0");
            USB.printf(" lift=%d", int(lift.triggered));
            USB.printf(" bump=%d,%d", int(bumperX.triggered), int(bumperY.triggered));
            USB.printf(" rain=%d", int(rain.triggered));
            USB.printf(" stop=%d", stop.triggered);
            USB.printf(" rightSp=0");
            USB.printf(" right=0");
            USB.printf(" rightPwm=0");
            USB.printf(" leftSp=0");
            USB.printf(" left=0");
            USB.printf(" leftPwm=0");
            USB.printf(" mowSp=0");
            USB.printf(" mow=0");
            USB.println(" mowPwm=0");
            nextInfoTime = now + INFOTIME;
            lps = 0;    
        }
    }
}