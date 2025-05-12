#include <Arduino.h>
#include "config.h"
#include "picodriver.h"
#include "test.h"

PicoDriver picoDriver;
// Test test;

void setup()
{
  USB.begin(USB_BAUDRATE);
  picoDriver.setup();
  // test.setup();
}

void loop()
{
  // test.run();
  picoDriver.run();
  // picoDriver.buzzer.run("imu calibration");
  // picoDriver.printLcd("Hello");
  // delay(1);
}

