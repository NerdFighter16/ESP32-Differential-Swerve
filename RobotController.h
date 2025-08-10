#pragma once

#include "JoystickProcessor.h"
#include "DiffSwerveModule.h"
#include "Utils.h"
#include "Config.h"
#include <IBusBM.h> 
#include <Bluepad32.h>     

class RobotController {
public:
    RobotController();
    void init();
    void runControlLoop();

private:
    DiffSwerveModule* modules[Config::numModules];  // use pointers
    JoystickProcessor joystickProcessor;

    float currentThetaEstimate;
    unsigned long lastLoopMicros;

    Utils::Vector3d calculateDesiredModuleState(
        int moduleIdx,
        const JoystickProcessor::Commands& commands,
        float robotTheta,
        const Utils::Vector3d& currentModuleState,
        float dt);

};
