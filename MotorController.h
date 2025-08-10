#pragma once

#include "Config.h"
#include <Arduino.h>

class MotorController {
public:
    MotorController(int topPwmPin, int topDirPin,
                    int bottomPwmPin, int bottomDirPin,
                    int topPwmChannel, int bottomPwmChannel);

    void init();
    void setVoltages(float voltageTop, float voltageBottom);

private:
    int pinTopPwm, pinTopDir;
    int pinBottomPwm, pinBottomDir;
    int channelTopPwm, channelBottomPwm;

    void setMotorOutput(float voltage, int dirPin, int pwmChannel);
};
