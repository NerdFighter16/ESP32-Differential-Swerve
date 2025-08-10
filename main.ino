#include <Arduino.h>
#include "RobotController.h"
#include <HardwareSerial.h>

HardwareSerial MySerial(2);
RobotController robot;

void setup() {
  Serial.begin(115200);
  Serial.println("Serial is working");
  robot.init();
  MySerial.begin(115200, SERIAL_8N1, 16, 17);
}

void loop() {
  robot.runControlLoop();
}
