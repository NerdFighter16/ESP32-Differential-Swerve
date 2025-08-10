#include "JoystickProcessor.h"
#include <Arduino.h> 
#include <cmath> 

JoystickProcessor::JoystickProcessor() {}

void JoystickProcessor::init() {
    filterVx.init(0.0f, Config::joystickEmaAlpha);
    filterVy.init(0.0f, Config::joystickEmaAlpha);
    filterW.init(0.0f, Config::joystickEmaAlpha);
    
    limiterVx.init(0.0f, Config::maxLinearAccel); 
    limiterVy.init(0.0f, Config::maxLinearAccel); 
    limiterW.init(0.0f, Config::maxAngularAccel); 
}


float JoystickProcessor::normalizeAndDeadband(int rawValue, int minVal, int maxVal, int midVal, float deadband) {
    float normalized = 0.0f;
    if (rawValue > midVal) {
        normalized = (float)(rawValue - midVal) / (maxVal - midVal);
    } else if (rawValue < midVal) {
        normalized = (float)(rawValue - midVal) / (midVal - minVal); 
    }
    
    if (fabs(normalized) < deadband) {
        return 0.0f;
    } else {
    
        if (normalized > 0) {
            return (normalized - deadband) / (1.0f - deadband);
        } else if (normalized < 0) {
             return (normalized + deadband) / (1.0f - deadband);
        } else {
            return 0.0f; 
        }
    }
}

void JoystickProcessor::mapSquareToCircle(float& x, float& y) {
    float x_out = x * sqrt(1.0f - y * y / 2.0f);
    float y_out = y * sqrt(1.0f - x * x / 2.0f);
    x = x_out;
    y = y_out;
}


float JoystickProcessor::shapeInputScalar(float value) {
    float sensitivity = constrain(Config::joystickShapingSensitivity, 0.0f, 1.0f);
    return sensitivity * powf(value, 3) + (1.0f - sensitivity) * value;
}


JoystickProcessor::Commands JoystickProcessor::process(int chVx_raw, int chVy_raw, int chW_raw, float dt) {
    float normVx = normalizeAndDeadband(chVx_raw, Config::ibusMin, Config::ibusMax, Config::ibusMid, Config::joystickDeadband);
    // Serial.print(normVx);
    // Serial.print("   ");     
    float normVy = normalizeAndDeadband(chVy_raw, Config::ibusMin, Config::ibusMax, Config::ibusMid, Config::joystickDeadband);
    //Serial.println(normVy);
    float normW = normalizeAndDeadband(chW_raw, Config::ibusMin, Config::ibusMax, Config::ibusMid, Config::joystickDeadband);

    mapSquareToCircle(normVx, normVy);

    float shapedVx = shapeInputScalar(normVx);
    float shapedVy = shapeInputScalar(normVy);
    float shapedW = shapeInputScalar(normW);

    float targetVx = shapedVx * Config::maxRobotSpeed;
    float targetVy = shapedVy * Config::maxRobotSpeed;
    float targetW = shapedW * Config::maxOmegaSpeed;

    float limitedVx = limiterVx.apply(targetVx, dt);
    float limitedVy = limiterVy.apply(targetVy, dt);
    float limitedW = limiterW.apply(targetW, dt);
    
    currentCommands.linearVelCmd.x = filterVx.apply(limitedVx);
    //Serial.print(currentCommands.linearVelCmd.x);
    //Serial.print("   ");
    currentCommands.linearVelCmd.y = filterVy.apply(limitedVy);
    currentCommands.linearVelCmd.y = -currentCommands.linearVelCmd.y;
    //Serial.println(currentCommands.linearVelCmd.y);
    currentCommands.angularVelCmd = filterW.apply(limitedW);
    //Serial.println(currentCommands.angularVelCmd);

    return currentCommands;
}