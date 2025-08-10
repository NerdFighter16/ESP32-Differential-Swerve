#pragma once

#include "Utils.h"
#include "Config.h"

class JoystickProcessor {
public:
    struct Commands {
        Utils::Vector2d linearVelCmd = {0.0f, 0.0f};  // vx, vy in m/s 
        float angularVelCmd = 0.0f;                   // omega in rad/s
    };

    JoystickProcessor();
    void init();
    Commands process(int chVx, int chVy, int chW, float dt);

private:
    Commands currentCommands;
    Utils::EmaFilter filterVx, filterVy, filterW;
    Utils::RateLimiter limiterVx, limiterVy, limiterW;

    float normalizeAndDeadband(int rawValue, int minVal, int maxVal, int midVal, float deadband);
    void mapSquareToCircle(float& x, float& y);
    float shapeInputScalar(float value);
};