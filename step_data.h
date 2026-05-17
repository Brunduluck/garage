#ifndef STEP_DATA_H
#define STEP_DATA_H

/**
 * @brief Represents a single kinematic step.
 * Note: In C++, 'typedef struct' is redundant. A simple 'struct Step_t' 
 * provides the same functionality.
 */
typedef struct
{
    double th = 0.0;
    double dt_sec = 0.0;
    double vT = 0.0;
    double Wz = 0.0;
    double dWz = 0.0;
    double angleZ = 0.0;
    double Ax = 0.0, Ay = 0.0;
    double Ux = 0.0, Uy = 0.0;
    double Vx = 0.0, Vy = 0.0;
    double Px = 0.0, Py = 0.0;
    double Bx = 0.0, By = 0.0;
    double Rx = 0.0, Ry = 0.0;
    long N = 0;
} Step_t;

#endif // STEP_DATA_H

#ifndef KALMAN_H
#define KALMAN_H

class Kalman {
public:
    Kalman();
    float getAngle(float newAngle, float newRate, float dt);

private:
    float Q_angle, Q_bias, R_measure;
    float angle, bias;
    float P[2][2];
};

class KalmanKinematic {
public:
    KalmanKinematic();
    float update(float raw_accel, float dt);
    
private:
    float Q_accel, R_obs, velocity, accel_bias, P;
};

#endif
