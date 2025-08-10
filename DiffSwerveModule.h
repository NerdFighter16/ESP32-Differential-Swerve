#pragma once

#include "ModuleSensors.h"
#include "MotorController.h"
#include "Utils.h"
#include "Config.h"
#include <QuickPID.h>

class DiffSwerveModule {
public:
    DiffSwerveModule(int moduleIdx,
                     int pinAziA, int pinAziB, int pinAziI,
                     int pinTopA, int pinTopB
                    //  int pinTopPwm, int pinTopDir,
                    //  int pinBotPwm, int pinBotDir,
                    //  int chanTopPwm, int chanBotPwm
                     );

    void init();
    void updateControl(const Utils::Vector3d& desiredState, int i, float dt);
    Utils::Vector3d updateAndGetCurrentState(float dt);
    float thetaA_target_prev = 0.0f;
    float smoothedThetaError = 0.0f;
    float theta_target_unwrapped = 0.0f;

public:
    Utils::Vector3d prevFinalDesiredState; // Made public for RobotController to access

private:
    int moduleIndex;
    ModuleSensors moduleSensors;
//    MotorController motorController;
    Utils::Matrix2x3 lqrGainK;
    Utils::Vector3d currentState;
    Utils::Vector3d finalDesiredState;
    float theta_measured;
    float theta_setpoint;
    float azimuth_power;
    float omegaW_measured;
    float omegaW_setpoint;
    float speed_power;

    QuickPID azimuthPID;
    QuickPID speedPID;
};