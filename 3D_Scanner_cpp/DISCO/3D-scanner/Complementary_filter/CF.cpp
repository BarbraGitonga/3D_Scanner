/*
 * CF.cpp
 *
 *  Created on: Jul 8, 2025
 *      Author: barbra gitonga (barbragitonga@gmail.com)
 */

#include "Complementary_filter/CF.h"

CF::CF() {
	state.phi_hat_rad = 0.0f;
	state.theta_hat_rad = 0.0f;
}

void CF::filter(const MPU6050Data& data, float alpha, float T_ms) {
    float dt = T_ms / 1000.0f;

    // Accelerometer estimates
    float phi_acc = atanf(data.accY / data.accZ);
    float theta_acc = asinf(data.accX / g);

    // Gyro-based integration
    float phi_dot = data.gyroX + tanf(state.theta_hat_rad) * (
    		sinf(state.phi_hat_rad) * data.gyroY + cosf(state.phi_hat_rad) * data.gyroZ);
    float theta_dot = (cosf(state.phi_hat_rad)) * data.gyroY - (sinf(state.phi_hat_rad) * data.gyroZ);

    // Combine accelerometer with integral of  gyro readings
    state.phi_hat_rad = (1.0f - alpha) * (state.phi_hat_rad + dt * phi_dot) + alpha * phi_acc;
    state.theta_hat_rad = (1.0f - alpha) * (state.theta_hat_rad + dt * theta_dot) + alpha * theta_acc;
}

AngleEstimate CF::getAngle() const {
    AngleEstimate angle;
    angle.roll = state.phi_hat_rad * RAD_TO_DEG;
    angle.pitch = state.theta_hat_rad * RAD_TO_DEG;
    return angle;
}
