#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "picodriver.h"

PicoDriver picoDriver;

void setup() {
  picoDriver.setup();
}

void loop() {
  picoDriver.run();
  //picoDriver.buzzer.run("imu calibration");
  //picoDriver.printLcd("Hello");
  //delay(1);
}