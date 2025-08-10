#include <Arduino.h>
#include <ESP32Encoder.h>
#include "Config.h"

namespace ShooterControl {

ESP32Encoder hoodEncoder;
int flywheelPwm = Config::flywheelPwmInit;
bool flywheelRunning = false;

void init() {
    pinMode(Config::flywheelPwmPin, OUTPUT);
    pinMode(Config::hoodPwmPin, OUTPUT);
    pinMode(Config::hoodDirPin, OUTPUT);
    pinMode(Config::flapPwmPin, OUTPUT);
    pinMode(Config::flapDirPin, OUTPUT);
    pinMode(Config::flapLimitShootPin, INPUT);
    pinMode(Config::flapLimitResetPin, INPUT);
    pinMode(Config::defensePwmPin, OUTPUT);
    pinMode(Config::defenseDirPin, OUTPUT);
    pinMode(Config::defenseLimitForwardPin, INPUT);
    pinMode(Config::defenseLimitBackPin, INPUT);

    hoodEncoder.attachFullQuad(Config::hoodEncA, Config::hoodEncB);
    hoodEncoder.setCount(0);
    ledcSetup(6, Config::pwmFrequency, Config::pwmResolutionBits);
    ledcAttachPin(Config::flywheelPwmPin, 6);
}

float getHoodAngleDeg() {
    int64_t ticks = hoodEncoder.getCount();
    float anglePerTick = 360.0f / Config::hoodEncPPR;
    return ticks * anglePerTick;
}

void updateFlywheel(uint16_t buttons) {
    if (buttons & Config::btnStart) flywheelRunning = !flywheelRunning;
    if (buttons & Config::btnA) flywheelPwm = max(0, flywheelPwm - Config::flywheelPwmIncrement);
    if (buttons & Config::btnY) flywheelPwm = min(255, flywheelPwm + Config::flywheelPwmIncrement);
    ledcWrite(6, flywheelRunning ? flywheelPwm : 0);
}


void updateHood(uint16_t buttons) {
    float angle = getHoodAngleDeg();
    if ((buttons & Config::btnB) && angle < Config::hoodMaxAngleDeg) {
        digitalWrite(Config::hoodDirPin, HIGH);
        analogWrite(Config::hoodPwmPin, 80);
    } else if ((buttons & Config::btnX) && angle > Config::hoodMinAngleDeg) {
        digitalWrite(Config::hoodDirPin, LOW);
        analogWrite(Config::hoodPwmPin, 80);
    } else {
        analogWrite(Config::hoodPwmPin, 0);
    }
}

void runFlapCycle(bool trigger) {
    static bool inCycle = false;
    static bool returning = false;

    if (trigger && !inCycle) {
        digitalWrite(Config::flapDirPin, HIGH);
        analogWrite(Config::flapPwmPin, 150);
        inCycle = true;
    } else if (inCycle && !returning && digitalRead(Config::flapLimitShootPin) == LOW) {
        digitalWrite(Config::flapDirPin, LOW);
        analogWrite(Config::flapPwmPin, 150);
        returning = true;
    } else if (inCycle && returning && digitalRead(Config::flapLimitResetPin) == LOW) {
        analogWrite(Config::flapPwmPin, 0);
        inCycle = false;
        returning = false;
    }
}

void runDefenseCycle(bool trigger) {
    static bool toggleState = false;
    static bool transitioning = false;

    if (trigger && !transitioning) {
        toggleState = !toggleState;
        transitioning = true;
    } else if (!trigger) {
        transitioning = false;
    }

    if (toggleState) {
        if (digitalRead(Config::defenseLimitForwardPin) == LOW) {
            analogWrite(Config::defensePwmPin, 0);
        } else {
            digitalWrite(Config::defenseDirPin, HIGH);
            analogWrite(Config::defensePwmPin, 150);
        }
    } else {
        if (digitalRead(Config::defenseLimitBackPin) == LOW) {
            analogWrite(Config::defensePwmPin, 0);
        } else {
            digitalWrite(Config::defenseDirPin, LOW);
            analogWrite(Config::defensePwmPin, 150);
        }
    }
}

void update(uint16_t buttons) {
    updateFlywheel(buttons);
    updateHood(buttons);
    runFlapCycle(buttons & Config::btnL2);
    runDefenseCycle(buttons & Config::btnL1);
}

} 
