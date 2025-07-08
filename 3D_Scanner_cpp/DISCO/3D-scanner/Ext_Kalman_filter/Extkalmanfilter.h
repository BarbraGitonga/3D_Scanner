/*
 * Extkalmanfilter.h
 *
 *  Created on: Jun 20, 2025
 *      Author: Barbra Gitonga (barbragitonga@gmail.com)
 */

#ifndef EXT_KALMAN_FILTER_H
#define EXT_KALMAN_FILTER_H

#include <math.h>
#include "MPU6050_driver/MPU6050.h"
#ifdef __cplusplus
extern "C" {
#endif

#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174533f
#define g ((float) 9.81f)

struct MPU6050Data {
	float accX, accY, accZ;
	float gyroX, gyroY, gyroZ;

	void setFrom(MPU6050_data& mpu_data) {
		accX  = mpu_data.acc_mps2[1];
		accY  = mpu_data.acc_mps2[0];
		accZ  = mpu_data.acc_mps2[2];
		gyroX = mpu_data.gyro_rad[1];
		gyroY = mpu_data.gyro_rad[0];
		gyroZ = mpu_data.gyro_rad[2];
	}
};


// In header file
struct AngleEstimate {
    float roll;
    float pitch;
};

typedef struct {
    float phi_rad; // roll(x[0])
    float theta_rad; // pitch(x[1])
    float bias_phi; // roll bias
    float bias_theta; // pitch bias
    float Q[4];  // Process noise covariance
	float R[3];  // Measurement noise covariance
	float P[16];  // Estimate covariance
} kal;

class ExtKalmanFilter {
public:
    ExtKalmanFilter(float Pinit ,float* Q_var, float* R_var, float phi_bias, float theta_bias);

    void predict(const MPU6050Data& gyro, float dt);
    void update (const MPU6050Data& accel);
    AngleEstimate getAngle() const;

private:
    kal state;

};

#ifdef __cplusplus
}
#endif

#endif // EXT_KALMAN_FILTER_H

