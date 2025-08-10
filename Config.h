#pragma once
#include <cmath>
#include <stdint.h> 

namespace Config {
    constexpr float nominalVoltage = 12.0f; 
    constexpr float wheelRadius = 0.0508f; 
    constexpr float maxRobotSpeed = 3.0f; 
    constexpr float maxOmegaSpeed = 2.0f * M_PI;

    constexpr int numModules = 3;
    struct ModulePosition { float x; float y; };
    constexpr ModulePosition modulePositions[numModules] = {
        {0.0f, 0.375f},
        {-0.265f, -0.265f},
        {0.265f, -0.265f}  
    };

    constexpr float loopPeriodSeconds = 0.01f; // 100hz
    constexpr uint32_t loopPeriodMicros = (uint32_t)(loopPeriodSeconds * 1000000);

    constexpr float diffSwerveWheelGearRatio = 5.3272f; 
    constexpr float diffSwerveAzimuthGearRatio = 18.2978f; 

    constexpr float joystickDeadband = 0.05f;       
    constexpr float joystickEmaAlpha = 0.3f;        // Smoothing factor
    constexpr float joystickShapingSensitivity = 0.5f;
    constexpr float maxLinearAccel = 2.0f;        
    constexpr float maxAngularAccel = M_PI;         
    constexpr int ibusMin = -512;                 
    constexpr int ibusMax = 508;                
    constexpr int ibusMid = -2;                 
    constexpr int ibusChannelVx = 1;             
    constexpr int ibusChannelVy = 0;             
    constexpr int ibusChannelW = 3;               
    constexpr int ibusPin = 27;                   

    constexpr float velocityEmaAlpha = 0.6f;        
    constexpr float accelEmaAlpha = 0.7f;         

    // Gains K for state error e = [thetaA_err, omegaA_err, omegaW_err]
    constexpr float lqrGainsK[6] =     { 0.67237, 0.0008, 0.19811,
 -0.67237, -0.0008, 0.19811};

    constexpr float minSafeVoltage = nominalVoltage * 0.75;       
    constexpr float pwmMinPercent = 8.0f;         
    constexpr float pwmMaxPercent = 94.0f;        
    constexpr uint32_t pwmMaxValue = 130;         
    constexpr float pwmZeroThreshold = 0.01f;     
    constexpr int pwmFrequency = 20000;           
    constexpr int pwmResolutionBits = 8;          

    // --- Pin Assignments ---

    // Module 0 Pins
    constexpr int m0AzimuthA = 35;              // Azimuth Encoder A
    constexpr int m0AzimuthB = 34;              // Azimuth Encoder B
    constexpr int m0AzimuthI = 2;              // Azimuth Encoder Index (Optional but recommended)
    constexpr int m0TopEncoderA = 4;           // Top Motor Encoder A
    constexpr int m0TopEncoderB = 5;           // Top Motor Encoder B

    constexpr int m1AzimuthA = 14;             
    constexpr int m1AzimuthB = 27;             
    constexpr int m1AzimuthI = 23;            
    constexpr int m1TopEncoderA = 32;          
    constexpr int m1TopEncoderB = 33 ;          

    // Module 2 Pins
    constexpr int m2AzimuthA = 13;             
    constexpr int m2AzimuthB = 12;             
    constexpr int m2AzimuthI = 21;             
    constexpr int m2TopEncoderA = 25;          
    constexpr int m2TopEncoderB = 26;          

    constexpr int azimuthEncoderPPR = 8192;     
    constexpr int topMotorEncoderPPR = 8192;     

    constexpr int flywheelPwmPin = 17;
    constexpr int flywheelPwmInit = 127; 
    constexpr int flywheelPwmIncrement = 10;

    constexpr int hoodPwmPin = 13;
    constexpr int hoodDirPin = 17;
    constexpr int hoodEncA = 18;
    constexpr int hoodEncB = 19;
    constexpr int hoodEncPPR = 8192;
    constexpr float hoodMaxAngleDeg = 55.0f;
    constexpr float hoodMinAngleDeg = 5.0f;

    constexpr int flapPwmPin = 16;
    constexpr int flapDirPin = 5;
    constexpr int flapLimitShootPin = 34;
    constexpr int flapLimitResetPin = 35;

    constexpr int defensePwmPin = 32;
    constexpr int defenseDirPin = 33;
    constexpr int defenseLimitForwardPin = 36;
    constexpr int defenseLimitBackPin = 39;

    constexpr uint16_t btnStart = 0x0004; 
    constexpr uint16_t btnA     = 0x0001; // decrease flywheel
    constexpr uint16_t btnY     = 0x0008; // increase flywheel
    constexpr uint16_t btnB     = 0x0002; // hood angle increase
    constexpr uint16_t btnX     = 0x0004; // hood angle decreaase
    constexpr uint16_t btnL2    = 0x0040; // shoot flap
    constexpr uint16_t btnL1    = 0x0010; // defense
}