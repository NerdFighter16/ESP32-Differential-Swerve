#include "ModuleSensors.h"
#include <cmath>
#include <Arduino.h>

#define PI 3.14159265358979323846f

ModuleSensors::ModuleSensors(int pinAziA, int pinAziB, int pinAziI, int pprAzi,
                             int pinTopA, int pinTopB, int pprTop)
    : prevThetaA(0.0f),
      prevThetaTop(0.0f)
{
    azimuthEncoder.attachFullQuad(pinAziA, pinAziB);
    azimuthEncoder.setCount(0);

    topEncoder.attachFullQuad(pinTopA, pinTopB);
    topEncoder.setCount(0);

    anglePerTickAzi = 2.0f * PI / pprAzi;
    anglePerTickTop = 2.0f * PI / pprTop;
}

void ModuleSensors::init() {
    filterOmegaA.init(0.0f, Config::velocityEmaAlpha);
    filterOmegaW.init(0.0f, Config::velocityEmaAlpha);
    filterOmegaTop.init(0.0f, Config::velocityEmaAlpha);
}

void ModuleSensors::homeAzimuth() {
    // No index support in ESP32Encoder
    Serial.println("Azimuth homing skipped (no index support).");
}

Utils::Vector3d ModuleSensors::updateAndGetCurrentState(float dt) {
    int64_t ticksAzi = azimuthEncoder.getCount();
    float thetaA = ticksAzi * anglePerTickAzi;
    float omegaA_measured = (thetaA - prevThetaA) / dt;
    prevThetaA = thetaA;

    int64_t ticksTop = topEncoder.getCount();
    float thetaTop = ticksTop * anglePerTickTop;
    float omegaTop_raw = (thetaTop - prevThetaTop) / dt;
    prevThetaTop = thetaTop;

    float omegaW_calculated = 0.0f;
    
    float omegaTop_filtered = filterOmegaTop.apply(omegaTop_raw);
    float omegaA_filtered = filterOmegaA.apply(omegaA_measured);


    if (fabsf(Config::diffSwerveAzimuthGearRatio) > 1e-6f) {
        omegaW_calculated = (2* omegaTop_filtered - (omegaA_measured * Config::diffSwerveAzimuthGearRatio))/ Config::diffSwerveWheelGearRatio;
    }

    //    float omegaW_filtered = filterOmegaW.apply(omegaW_calculated);
    currentState.x = thetaA;
    currentState.y = omegaA_filtered;
    currentState.z = omegaW_calculated;

    return currentState;
}