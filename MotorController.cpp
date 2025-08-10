#include "MotorController.h"

MotorController::MotorController(int topPwmPin, int topDirPin,
                                 int bottomPwmPin, int bottomDirPin,
                                 int topPwmChannel, int bottomPwmChannel)
    : pinTopPwm(topPwmPin), pinTopDir(topDirPin),
      pinBottomPwm(bottomPwmPin), pinBottomDir(bottomDirPin),
      channelTopPwm(topPwmChannel), channelBottomPwm(bottomPwmChannel)
{}

void MotorController::init() {
    pinMode(pinTopPwm, OUTPUT);
    pinMode(pinTopDir, OUTPUT);
    pinMode(pinBottomPwm, OUTPUT);
    pinMode(pinBottomDir, OUTPUT);

    ledcSetup(channelTopPwm, 20000, 8); 
    ledcAttachPin(pinTopPwm, channelTopPwm);

    ledcSetup(channelBottomPwm, 20000, 8);
    ledcAttachPin(pinBottomPwm, channelBottomPwm);
}

void MotorController::setVoltages(float voltageTop, float voltageBottom) {
    setMotorOutput(voltageTop, pinTopDir, channelTopPwm);
    setMotorOutput(voltageBottom, pinBottomDir, channelBottomPwm);
}

void MotorController::setMotorOutput(float voltage, int dirPin, int pwmChannel) {
    bool dir = (voltage >= 0);
    float absVoltage = fabs(voltage);

    if (absVoltage < Config::pwmZeroThreshold * Config::nominalVoltage) {
        digitalWrite(dirPin, LOW);
        ledcWrite(pwmChannel, 0);
    } else {
        digitalWrite(dirPin, dir ? HIGH : LOW);
        float duty = absVoltage / Config::nominalVoltage;
        duty = constrain(duty, Config::pwmMinPercent/100.0f, Config::pwmMaxPercent/100.0f);
        ledcWrite(pwmChannel, (uint8_t)(duty * Config::pwmMaxValue));
//        Serial.println(duty * Config::pwmMaxValue);
    }
}
