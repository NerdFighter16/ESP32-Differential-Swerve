#include "DiffSwerveModule.h"
#include "MotorController.h"
#include <QuickPID.h>
#include <HardwareSerial.h>

extern HardwareSerial MySerial;

DiffSwerveModule::DiffSwerveModule(int moduleIdx,
                                   int pinAziA, int pinAziB, int pinAziI,
                                   int pinTopA, int pinTopB
                                //    int pinTopPwm, int pinTopDir,
                                //    int pinBotPwm, int pinBotDir,
                                //    int chanTopPwm, int chanBotPwm
                                   )
    : moduleIndex(moduleIdx),
      moduleSensors(pinAziA, pinAziB, pinAziI, Config::azimuthEncoderPPR,
                    pinTopA, pinTopB, Config::topMotorEncoderPPR),
      //motorController(pinTopPwm, pinTopDir, pinBotPwm, pinBotDir, chanTopPwm, chanBotPwm),
      lqrGainK(Config::lqrGainsK),
      theta_measured(0.0f), theta_setpoint(0.0f), azimuth_power(0.0f),
      omegaW_measured(0.0f), omegaW_setpoint(0.0f), speed_power(0.0f),
      azimuthPID(&theta_measured, &azimuth_power, &theta_setpoint),
      speedPID(&omegaW_measured, &speed_power, &omegaW_setpoint)
{}

void DiffSwerveModule::init() {
    moduleSensors.init();
    moduleSensors.homeAzimuth();
    //motorController.init()
    azimuthPID.SetTunings(60.0f, 0.005f, 0.3f);  // Kp, Ki, Kd 
    azimuthPID.SetOutputLimits(-Config::pwmMaxValue, Config::pwmMaxValue);
    azimuthPID.SetMode(QuickPID::Control::automatic);

    speedPID.SetTunings(0.2f, 0.00002f, 0.01f);    // Kp, Ki, Kd 
    speedPID.SetOutputLimits(-Config::pwmMaxValue, Config::pwmMaxValue);
    speedPID.SetMode(QuickPID::Control::automatic);


}


Utils::Vector3d DiffSwerveModule::updateAndGetCurrentState(float dt) {
    currentState = moduleSensors.updateAndGetCurrentState(dt);
    return currentState;
}

void setVoltages(int moduleIndex, float voltageTop, float voltageBottom){
    MySerial.print(moduleIndex);
    MySerial.print(',');
    MySerial.print(voltageTop, 3);
    MySerial.print(',');
    MySerial.print(voltageBottom, 3);
    MySerial.print('\n');
}

void DiffSwerveModule::updateControl(const Utils::Vector3d& desiredState, int i, float dt) {
    finalDesiredState = desiredState;

    
    float theta_unwrapped_target = 0.0f;
    theta_measured = currentState.x;
    theta_setpoint = finalDesiredState.x;
    float theta_error = finalDesiredState.x - currentState.x;
    Serial.print(theta_error);
    azimuthPID.Compute();


//    theta_unwrapped_target = Utils::unwrapAngle(theta_unwrapped_target, finalDesiredState.x);
//    theta_error = theta_unwrapped_target - currentState.x;


    omegaW_setpoint = finalDesiredState.z;
    omegaW_measured = currentState.z;
    speedPID.Compute();

    float pwmTop = speed_power + azimuth_power;
    float pwmBot = (speed_power - azimuth_power);
    

    // NEW CODE
    // Calculate the optimized error and wheel direction internally.
    //finalDesiredState is globally available 
/*
    float wheel_direction;
    theta_error = finalDesiredState.x - currentState.x;
//    theta_error = Utils::shortestAngleDist(finalDesiredState.x, currentState.x, wheel_direction);

    theta_setpoint = 0.0f;
    azimuthPID.Compute();
    omegaW_setpoint = finalDesiredState.z * wheel_direction;
    omegaW_measured = currentState.z;
    speedPID.Compute();

    float pwmTop = speed_power + azimuth_power;
    float pwmBot = -(speed_power - azimuth_power);
*/

/*
    // Apply PWM normalization
    float max_abs_output = fmax(fabs(pwmTop), fabs(pwmBot));
    if (max_abs_output < 1e-6f) { // Guard against division by zero
        pwmTop = 0.0f;
        pwmBot = 0.0f;
    } else {
        float scale_factor = fmin(1.0f, Config::pwmMaxPercent / max_abs_output);
        pwmTop *= scale_factor;
        pwmBot *= scale_factor;
    }
*/
//    pwmTop = map(PWM_top, -Config::maxPwmPercent, Config::maxPwmPercent, -255, 255);
//    pwmBot = map(PWM_top, -Config::maxPwmPercent, Config::maxPwmPercent, -255, 255);


//was changed to accept pwm duty cycle in slave esp
    setVoltages(moduleIndex,pwmTop , pwmBot );
    
    if(moduleIndex == 1){    
        Serial.print("Final Desired Theta :");
        Serial.print(finalDesiredState.x);
        Serial.print("  ");
        Serial.print("Current Theta :");
        Serial.print(currentState.x) ;
        Serial.print("  ");
    //    Serial.print(speed_power);
    //    Serial.print("   ");
    //    Serial.print("WrapAngle");
    //    Serial.print(Utils::wrapAngle(finalDesiredState.x - currentState.x));
        Serial.print("Theta Measured");
        Serial.print(theta_measured);
           Serial.print("   ");
    //    Serial.print(azimuth_power);
    //    Serial.print("   ");
        Serial.print("PWM Top :");
        Serial.print(pwmTop);
        Serial.print("Pwm bottom");
        Serial.print("   ");
        Serial.println(pwmBot);
    }
    

    prevFinalDesiredState = finalDesiredState;
}

/*
void DiffSwerveModule::updateControl(const Utils::Vector3d& desiredState,int i, float dt) {
    finalDesiredState = desiredState; 

    Utils::Vector3d error;
    error.x = -Utils::shortestAngleDist(finalDesiredState.x, currentState.x);
    error.y = -(finalDesiredState.y - currentState.y);
    error.z = -(finalDesiredState.z - currentState.z);

    Utils::Vector2d control =  Utils::multiplyLqr(lqrGainK, error);

    float Vtop = control.x;
    float Vbot = control.y;

  float voltageCompFactor = Config::nominalVoltage / railVoltage;
    Vtop *= voltageCompFactor;
    Vbot *= voltageCompFactor;


    setVoltages(i, Vtop, Vbot);
    if (moduleIndex == 1) {
   Serial.print("Vtop: ");
   Serial.print(Vtop);
   Serial.print("  Vbot: ");
   Serial.println(Vbot);
}

    prevFinalDesiredState = finalDesiredState;
}

    float applyDeadzone(float voltage, float threshold ) {
    if (fabs(voltage) < threshold)
        return 0.0f;
    return (voltage > 0 ? 1 : -1) * max(threshold, fabs(voltage));
}


}
*/

/*
   float applyDeadzone(float voltage, float threshold ) {
    if (fabs(voltage) < threshold)
        return 0.0f;
    return (voltage > 0 ? 1 : -1) * max(threshold, fabs(voltage));
    }

    void DiffSwerveModule::updateControl(const Utils::Vector3d& desiredState, int i, float dt) {
    finalDesiredState = desiredState;



    Utils::Vector3d error;
    error.x = Utils::shortestAngleDist(finalDesiredState.x, currentState.x); 
    error.y = finalDesiredState.y - currentState.y; 
    error.z = finalDesiredState.z - currentState.z; 

    const float Gwheel = Config::diffSwerveWheelGearRatio;
    const float Gazi = Config::diffSwerveAzimuthGearRatio;

    float omegaBottom = finalDesiredState.z * Gwheel - finalDesiredState.y * Gazi;


    float Kp_theta = 5.877f;
    float Ki_theta = 0.81f;
    float Kd_theta = 1.62f;

    static float theta_integral[3] = {0.0f};
    static float theta_prev_error[3] = {0.0f};


    float theta_error = Utils::shortestAngleDist(finalDesiredState.x, currentState.x);

    theta_integral[i] += theta_error * dt;
    float theta_derivative = (theta_error - theta_prev_error[i]) / dt;
    theta_prev_error[i] = theta_error;

    float omegaTop_theta_correction = Kp_theta * theta_error * 2.0f * Gazi + Ki_theta * theta_integral[i] + Kd_theta * theta_derivative;

    float omegaTop_desired = finalDesiredState.z * Gwheel + finalDesiredState.y * Gazi + omegaTop_theta_correction;

    float Vtop = (omegaTop_desired * Config::nominalVoltage/350) + 1.0f ;
    float Vbot = -(omegaBottom  * Config::nominalVoltage / 350 ) + 1.0f;
    float minEffectiveTorque = 2.0f;

//    if (fabs(theta_error) > 0.1f) {  
//        float sign = (theta_error > 0 ? 1.0f : -1.0f);
//        Vtop += sign * minEffectiveTorque;
//    }

    Vtop = applyDeadzone(Vtop , 1.0f);
    Vbot = applyDeadzone(Vbot , 1.0f);



    float maxVoltage = 12.0f;
    Vtop = constrain(Vtop, -maxVoltage, maxVoltage);
    Vbot = constrain(Vbot, -maxVoltage, maxVoltage);

    if(moduleIndex == 1){
        Serial.print(omegaTop_desired);
        Serial.print("   ");
        Serial.print(error.x);
        Serial.print("   ");
        Serial.print(omegaBottom);
        Serial.print("   ");
        Serial.print(Vtop);
        Serial.print("   ");
        Serial.println(Vbot);
    }
    setVoltages(moduleIndex,Vtop, Vbot);

    prevFinalDesiredState = finalDesiredState;
}

*/

/*
    float applyDeadzone(float voltage, float threshold ) {
    if (fabs(voltage) < threshold)
        return 0.0f;
    return (voltage > 0 ? 1 : -1) * max(threshold, fabs(voltage));
    }

    void DiffSwerveModule::updateControl(const Utils::Vector3d& desiredState, int i, float dt) {
    finalDesiredState = desiredState;

    Utils::Vector3d error;
    error.x = Utils::shortestAngleDist(finalDesiredState.x, currentState.x); 
    error.y = finalDesiredState.y - currentState.y; 
    error.z = finalDesiredState.z - currentState.z; 

    const float Gwheel = Config::diffSwerveWheelGearRatio;
    const float Gazi = Config::diffSwerveAzimuthGearRatio;

    float omegaBottom = finalDesiredState.z * Gwheel - finalDesiredState.y * Gazi;

    static float integral[3] = {0}; 
    static float prevError[3] = {0};

    float Kp_theta = 5.877f;
    float Ki_theta = 0.81f;
    float Kd_theta = 1.62f;

    static float theta_integral[3] = {0.0f};
    static float theta_prev_error[3] = {0.0f};


    float theta_error = Utils::shortestAngleDist(finalDesiredState.x, currentState.x);

    theta_integral[i] += theta_error * dt;
    float theta_derivative = (theta_error - theta_prev_error[i]) / dt;
    theta_prev_error[i] = theta_error;

    float omegaTop_theta_correction = Kp_theta * theta_error * 2.0f * Gazi + Ki_theta * theta_integral[i] + Kd_theta * theta_derivative;

    float omegaTop_desired = finalDesiredState.z * Gwheel + finalDesiredState.y * Gazi + omegaTop_theta_correction;

    float Vtop = (omegaTop_desired * Config::nominalVoltage/350) + 1.0f ;
    float Vbot = -(omegaBottom  * Config::nominalVoltage / 350 ) + 1.0f;
    float minEffectiveTorque = 2.0f;

//    if (fabs(theta_error) > 0.1f) {  
//        float sign = (theta_error > 0 ? 1.0f : -1.0f);
//        Vtop += sign * minEffectiveTorque;
//    }

    Vtop = applyDeadzone(Vtop , 1.0f);
    Vbot = applyDeadzone(Vbot , 1.0f);



    float maxVoltage = 12.0f;
    Vtop = constrain(Vtop, -maxVoltage, maxVoltage);
    Vbot = constrain(Vbot, -maxVoltage, maxVoltage);

    if(moduleIndex == 1){
        Serial.print(omegaTop_desired);
        Serial.print("   ");
        Serial.print(error.x);
        Serial.print("   ");
        Serial.print(omegaBottom);
        Serial.print("   ");
        Serial.print(Vtop);
        Serial.print("   ");
        Serial.println(Vbot);
    }
    setVoltages(moduleIndex,Vtop, Vbot);

    prevFinalDesiredState = finalDesiredState;
}
*/


/*

void DiffSwerveModule::updateControl(const Utils::Vector3d& desiredState, int i, float dt){
    finalDesiredState = desiredState;

    Utils::Vector3d error;

    error.x = Utils::shortestAngleDist(finalDesiredState.x, currentState.x);
    error.y = finalDesiredState.y - currentState.y;
    error.z = finalDesiredState.z - currentState.z;

    float k_Theta_A = 1.577f;
    float k_Omega_A = 0.11f;
    float k_Omega_W = 0.0f;

    float Vtop = (error.x * k_Theta_A + error.y * k_Omega_A + error.z * k_Omega_W) ;
    float Vbot = (error.x * k_Theta_A + error.y * k_Omega_A - error.z * k_Omega_W ) ;

    setVoltages(moduleIndex, Vtop, Vbot);

    prevFinalDesiredState = finalDesiredState;
}
*/

/*
void DiffSwerveModule::updateControl(const Utils::Vector3d& desiredState, int i, float dt) {
    finalDesiredState = desiredState; 

    Utils::Vector3d error;
    error.x = Utils::shortestAngleDist(finalDesiredState.x, currentState.x);
    error.y = finalDesiredState.y - currentState.y;
    error.z = finalDesiredState.z - currentState.z;
//    if(i == 1){
//        Serial.print(error.x);
//        Serial.println(currentState.x);
//    }
//    if(i == 0){
//        Serial.print(error.x);
//        Serial.print("   ");
//        Serial.print(error.y);
//        Serial.print("   ");
//        Serial.println(error.z);
//    }
//    if (moduleIndex == 1) {
//    moduleSensors.printDebug(moduleIndex);
//    }
    float k_theta = 0.0f;
    float k_omegaA = 0.0f;
    float k_omegaW = 0.0f;

    if(moduleIndex == 0){
    k_theta = 1.26774f;
    k_omegaA = 0.0f;
    k_omegaW = 0.00000002f;
    }

    if(moduleIndex == 1){
    k_theta = 0.84774f;
    k_omegaA = 0.17947f;
    k_omegaW = 0.01f;
    }

    if(moduleIndex == 2){
    k_theta = 1.20774f;
    k_omegaA = 0.0f;
    k_omegaW = 0.0000005f;
    }

    float Vtop = - (k_theta * (error.x) + k_omegaA * error.y + k_omegaW * error.z);
    float Vbot =   (k_theta * (error.x) + k_omegaA * error.y - k_omegaW * error.z);

    float maxControlVoltage = 12.0f;
    Vtop = constrain(Vtop, -maxControlVoltage, maxControlVoltage);
    Vbot = constrain(Vbot, -maxControlVoltage, maxControlVoltage);


//    float voltageCompFactor = map(Config::nominalVoltage, ;
//    Vtop *= voltageCompFactor;
//    Vbot *= voltageCompFactor;



    setVoltages(moduleIndex, Vtop, Vbot);
//if (moduleIndex == 0) {
//   Serial.print("Module 0 | Vtop: ");
//   Serial.print(Vtop);
//   Serial.print("  Vbot: ");
//   Serial.println(Vbot);
//}

    prevFinalDesiredState = finalDesiredState;
}
*/