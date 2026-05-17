#include <M5CoreS3.h>
#include "step_data.h"

Kalman::Kalman() {
        Q_angle   = 0.0001f;   // Highly trust the gyro path
        Q_bias    = 0.0003f;   
        R_measure = 0.9000f;   // Strongly damp out shaky accelerometer spikes
        angle = 0.0f; 
        bias = 0.0f;
        P[0][0] = 0.1f; P[0][1] = 0.0f;
        P[1][0] = 0.0f; P[1][1] = 0.1f;
    };

float Kalman::getAngle(float newAngle, float newRate, float dt)
{
        // Step 1: Predict
        float rate = newRate - bias;
        angle += dt * rate;

        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Step 2: Update
        float S = P[0][0] + R_measure; 
        float K[2]; 
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        float y = newAngle - angle; 
        angle += K[0] * y;
        bias  += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    };

KalmanKinematic::KalmanKinematic() {
        Q_accel = 0.01f; 
        R_obs = 0.1f;    // Observation noise
        velocity = 0.0f;
        accel_bias = 0.0f;
        P = 1.0f;
    };

// Simple 1D Kalman to smooth acceleration and estimate velocity
float KalmanKinematic::update(float raw_accel, float dt) {
        // Predict velocity
        velocity += (raw_accel - accel_bias) * dt;
        P += Q_accel;

        // In a pure IMU system, velocity drifts. 
        // We apply a small damping factor (Pseudo-measurement of 0) 
        // to keep velocity from drifting when the device is still.
        float K = P / (P + R_obs);
        // velocity = velocity + K * (0.0f - velocity) * 0.01f; // Damping
        velocity = velocity + K * (0.0f - velocity) * 0.1f; // Increased from 0.01f
        P = (1 - K) * P;

        return velocity;
    };
