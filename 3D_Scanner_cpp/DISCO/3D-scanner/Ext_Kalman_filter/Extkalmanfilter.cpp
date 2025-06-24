/*
 * Extkalmanfilter.cpp
 *
 *  Created on: Jun 20, 2025
 *      Author: Barbra Gitonga (barbragitonga@gmail.com)
 *
 */

#include <Ext_Kalman_filter/Extkalmanfilter.h>

ExtKalmanFilter::ExtKalmanFilter() {
    // Constructor
	Q_angle = 0.001f;
	Q_bias = 0.003f;
	R_measure = 0.03f;

	angle = 0.0f;
	bias = 0.0f;

	P[0][0] = 1.0f; P[0][1] = 0.0f;
	P[1][0] = 0.0f; P[1][1] = 1.0f;
}

void ExtKalmanFilter::setAngle(float newAngle) {
    angle = newAngle;
}

float ExtKalmanFilter::getAngle() {
    return angle;
}

void ExtKalmanFilter::changeR(float NewR_measure) {
    R_measure =  NewR_measure;
}

float ExtKalmanFilter::getBias() {
    return bias;
}

float ExtKalmanFilter::update(float raw_accel, float raw_gyro, float dt) {
    // Step 1: Predict the angle
    float rate = raw_gyro - bias;
    angle += dt * rate;

    // Step 2: Update estimation error covariance
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Step 3: Innovation (measurement residual)
    float y = raw_accel - angle;

    // Step 4: Innovation covariance
    float S = P[0][0] + R_measure;

    // Step 5: Kalman Gain
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Step 6: Update state estimate
    angle += K[0] * y;
    bias  += K[1] * y;

    // Step 7: Update error covariance matrix
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}
