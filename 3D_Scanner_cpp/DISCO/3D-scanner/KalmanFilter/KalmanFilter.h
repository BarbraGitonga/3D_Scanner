/*
 * Extkalmanfilter.h
 *
 *  Created on: Jun 20, 2025
 *      Author: lenovo
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

class KalmanFilter {
public:
    KalmanFilter();
    float update(float raw_accel, float raw_gyro, float dt); // dt in seconds
    void setAngle(float angle);
    void changeR(float NewR_measure);
    float getAngle();
    float getBias();

private:
    float angle = 0.0f;
    float bias = 0.0f;

    float P[2][2] = {{1.0, 0.0}, {0.0, 1.0}};
    float Q_angle = 0.001f;
    float Q_bias = 0.003f;
    float R_measure = 0.03f;
};

#ifdef __cplusplus
}
#endif

#endif // EXT_KALMAN_FILTER_H

