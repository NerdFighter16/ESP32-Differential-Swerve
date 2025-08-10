#pragma once
#include <cmath>


namespace Utils {
    inline float wrapAngle(float angle) {
        return atan2f(sinf(angle), cosf(angle));
    }

    inline float shortestAngleDist(float targetAngle, float currentAngle, float& wheel_direction) {
        float error = wrapAngle(targetAngle - currentAngle);
        if(fabs(error) > (M_PI / 2.0f)){
            error = error - copysignf(M_PI, error);
            wheel_direction = -1.0f;
        }else{
            wheel_direction = 1.0f;
        }
        return error;
    }
    
    
    inline float unwrapAngle(float previousAngle, float newAngle){
        float diff = newAngle - previousAngle;
        while(diff >  M_PI){
            diff -= 2 * M_PI;
        }
        while(diff < -M_PI){
            diff += 2 * M_PI;
        }
        previousAngle += diff;
        return previousAngle;
    }

    struct EmaFilter {
        float alpha = 0.5f;
        float previousOutput = 0.0f;
        void init(float initialValue = 0.0f, float filterAlpha = 0.5f) {
            previousOutput = initialValue;
            alpha = filterAlpha;
        }
        float apply(float input) {
            previousOutput = alpha * input + (1.0f - alpha) * previousOutput;
            return previousOutput;
        }
    };

    struct RateLimiter {
        float maxRatePerSecond = 3.0f;
        float previousOutput = 0.0f;
        void init(float initialValue = 0.0f, float rate = 3.0f) {
            previousOutput = initialValue;
            maxRatePerSecond = rate;
        }
        float apply(float input, float dt) {
            float maxChange = maxRatePerSecond * dt;
            float change = input - previousOutput;
            float clampedChange = fmaxf(-maxChange, fminf(maxChange, change)); 
            previousOutput = previousOutput + clampedChange;
            return previousOutput;
        }
    };

    struct Vector2d { 
        float x = 0.0f; 
        float y = 0.0f; 

        Vector2d() = default; 

        Vector2d(float x_val, float y_val) : x(x_val), y(y_val) {}
    };

    
    struct Vector3d { 
        float x = 0.0f; 
        float y = 0.0f; 
        float z = 0.0f; 

        Vector3d() = default;
        Vector3d(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}
    };

    struct Matrix2x3 {
        float data[6] = {0};
        Matrix2x3(const float* gains) {
            for (int i = 0; i < 6; ++i) {
                data[i] = gains[i];
            }
        }
        Matrix2x3() = default;
    };

    inline Vector2d multiplyLqr(const Matrix2x3& gainK, const Vector3d& errorE) {
        return Vector2d(
            -(gainK.data[0]*errorE.x + gainK.data[1]*errorE.y + gainK.data[2]*errorE.z),
            -(gainK.data[3]*errorE.x + gainK.data[4]*errorE.y + gainK.data[5]*errorE.z)
        );
    }
}