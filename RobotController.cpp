#include "RobotController.h"
#include <Arduino.h> 
#include <cmath>     
#include <Bluepad32.h>
#include "gamepad.h"

#define PI 3.14159265358979323846f 

RobotController::RobotController()
    : currentThetaEstimate(0.0f),
      lastLoopMicros(0)
{
    for (int i = 0; i < Config::numModules; ++i)
        modules[i] = nullptr;
}


void RobotController::init() {
    Serial.begin(115200);  
    Serial.println("Robot init");
    delay(1000);
    Gamepad_setup();                 // Init Gamepad
    Serial.println("Gamepad setup working");
    delay(1000);
    joystickProcessor.init();
    Serial.println("Joystickprocessor initialalized");
    delay(1000);

    // Now initialize modules (safe after setup)
    modules[0] = new DiffSwerveModule(0,
        Config::m0AzimuthA, Config::m0AzimuthB, Config::m0AzimuthI,
        Config::m0TopEncoderA, Config::m0TopEncoderB
        // Config::m0TopPwm, Config::m0TopDir,
        // Config::m0BottomPwm, Config::m0BottomDir,
        // Config::m0TopPwmChannel, Config::m0BottomPwmChannel
        );

    modules[1] = new DiffSwerveModule(1,
        Config::m1AzimuthA, Config::m1AzimuthB, Config::m1AzimuthI,
        Config::m1TopEncoderA, Config::m1TopEncoderB
        // Config::m1TopPwm, Config::m1TopDir,
        // Config::m1BottomPwm, Config::m1BottomDir,
        // Config::m1TopPwmChannel, Config::m1BottomPwmChannel
        );


    modules[2] = new DiffSwerveModule(2,
        Config::m2AzimuthA, Config::m2AzimuthB, Config::m2AzimuthI,
        Config::m2TopEncoderA, Config::m2TopEncoderB
        // Config::m2TopPwm, Config::m2TopDir,
        // Config::m2BottomPwm, Config::m2BottomDir,
        // Config::m2TopPwmChannel, Config::m2BottomPwmChannel
        );

    Serial.print("Modules initialised");

    for (int i = 0; i < Config::numModules; i++) {
        modules[i]->init();
    }

    lastLoopMicros = micros();
    Serial.println("Robot Initialized.");
}

float matchToSame2PiBand(float measured, float target) {
    // Bring target close to measured within ±π
    float delta = target - measured;
    delta = fmodf(delta + M_PI, 2 * M_PI);
    if (delta < 0) delta += 2 * M_PI;
    delta -= M_PI;

    return measured + delta;
}

void RobotController::runControlLoop() {
    unsigned long now = micros();
    float dt = (now - lastLoopMicros) / 1e6f;
    if (dt <= 0.0f || dt > Config::loopPeriodSeconds * 2.0f) {
        dt = Config::loopPeriodSeconds;
    }
    lastLoopMicros = now;

    Gamepad_update();  // update controller state

    ControllerPtr ctl = Gamepad_getConnected();

    int chVx = ctl ? ctl->axisRX() : 0;
//    Serial.println(chVx);
    int chVy = ctl ? ctl->axisRY() : 0;
//    chVy = - chVy;
//    Serial.println(chVy);
    int chW  = ctl ? ctl->axisX() : 0;
//    Serial.println(chW);

    JoystickProcessor::Commands commands = joystickProcessor.process(
        chVx, chVy, chW, dt
    );

    for (int i = 0; i < Config::numModules; i++) {
        Utils::Vector3d currentState = modules[i]->updateAndGetCurrentState(dt);
        Utils::Vector3d desiredState = calculateDesiredModuleState(i, commands, currentThetaEstimate, currentState, dt);
        modules[i]->updateControl(desiredState, i, dt);
    }
}



float computeOmegaAzimuth(float vx, float vy, float dt) {
    static float lastVx = 0.0f;
    static float lastVy = 0.0f;

    float deltaVx = vx - lastVx;
    float deltaVy = vy - lastVy;

    float denominator = vx * vx + vy * vy;
    float omegaAzimuth = 0.0f;

    if (fabs(denominator) > 1e-6f && dt > 1e-6f) {
        omegaAzimuth = (vx * deltaVy/dt - vy * deltaVx/dt) / (denominator);
    }

    lastVx = vx;
    lastVy = vy;

    return omegaAzimuth;
}

Utils::Vector3d RobotController::calculateDesiredModuleState(
    int moduleIdx,
    const JoystickProcessor::Commands& commands, 
    float robotTheta ,
    const Utils::Vector3d& currentModuleState,
    float dt)
{
    float cosTheta = cosf(robotTheta);
    float sinTheta = sinf(robotTheta);
//robot-to-field-centric
    
    float vxRobot = commands.linearVelCmd.x * cosTheta + commands.linearVelCmd.y * sinTheta;
    float vyRobot = -commands.linearVelCmd.x * sinTheta + commands.linearVelCmd.y * cosTheta; 
    float omegaRobot = commands.angularVelCmd;

    Config::ModulePosition modulePos = Config::modulePositions[moduleIdx];

    float wheelVx = vxRobot - omegaRobot * modulePos.y;
//    Serial.print(wheelVx);
//    Serial.print("   ");
    float wheelVy = vyRobot + omegaRobot * modulePos.x;
//    Serial.print(wheelVy);

    float wheelLinearSpeed = sqrt(wheelVx *wheelVx + wheelVy *wheelVy);


    //if (moduleIdx == 2){
    //    Serial.println(thetaA_target);
    //} 

        //if (thetaA_target < 0){
        //    omegaW_target = -omegaW_target;
        //} 
    //    if(moduleIdx==0){
    //    Serial.print(omegaW_target);
    //    Serial.print("   ");
    //    }
    float omegaW_target = sqrtf(wheelVx * wheelVx + wheelVy * wheelVy) / Config::wheelRadius;
    //aryan
    float rawTheta = atan2f(wheelVy, wheelVx);
    //thetaA_target = currentModuleState.x; // default: hold current angle
    float omegaA_target = 0.0f;
    float thetaA_target = 0.0f;
    
    if (fabs(omegaW_target) >= 0.1f) {
        float rawTheta = atan2f(wheelVy, wheelVx);
        thetaA_target = matchToSame2PiBand(currentModuleState.x, rawTheta);
        modules[moduleIdx]->prevFinalDesiredState.x = thetaA_target;

        omegaA_target = computeOmegaAzimuth(wheelVx, wheelVy, dt);
    }

    else{
        thetaA_target = modules[moduleIdx]->prevFinalDesiredState.x;
    }
    return Utils::Vector3d{thetaA_target, omegaA_target, omegaW_target};
}




    