#pragma once

#include "Utils.h"
#include "Config.h"
#include <ESP32Encoder.h>

class ModuleSensors {
public:
    ModuleSensors(int pinAziA, int pinAziB, int pinAziI, int pprAzi,
                  int pinTopA, int pinTopB, int pprTop);

    void init();
    Utils::Vector3d updateAndGetCurrentState(float dt);
    void homeAzimuth();

private:
    ESP32Encoder azimuthEncoder;
    ESP32Encoder topEncoder;

    float prevThetaA;
    float prevThetaTop;
    float anglePerTickAzi;
    float anglePerTickTop;

    Utils::EmaFilter filterOmegaA;
    Utils::EmaFilter filterOmegaW;
    Utils::EmaFilter filterOmegaTop;

    Utils::Vector3d currentState;
};