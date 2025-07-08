/*
 * CF.h
 *
 *  Created on: Jul 8, 2025
 *      Author: Barbra Gitonga(barbragitonga@gmail.com)
 */

#ifndef CF_H_
#define CF_H_

#include <math.h>
#include "MPU6050_driver/MPU6050.h"
#ifdef __cplusplus
extern "C" {
#endif

#define g ((float)9.81f)
#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174533f

struct AngleEstimate {
    float roll;
    float pitch;
};

struct MPU6050Data {
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float gyrPrev[3] = {0.0f};
    float accPrev[3] = {0.0f};

    void setFrom(const MPU6050_data& mpu_data, float LPF_GYR_ALPHA, float LPF_ACC_ALPHA) {
        gyroX = (LPF_GYR_ALPHA * gyrPrev[0] + (1.0f - LPF_GYR_ALPHA) * mpu_data.gyro_rad[0]);
        gyroY = (LPF_GYR_ALPHA * gyrPrev[1] + (1.0f - LPF_GYR_ALPHA) * mpu_data.gyro_rad[1]);
        gyroZ = LPF_GYR_ALPHA * gyrPrev[2] + (1.0f - LPF_GYR_ALPHA) * mpu_data.gyro_rad[2];

        accX = LPF_ACC_ALPHA * accPrev[0] + (1.0f - LPF_ACC_ALPHA) * mpu_data.acc_mps2[0];
        accY = LPF_ACC_ALPHA * accPrev[1] + (1.0f - LPF_ACC_ALPHA) * mpu_data.acc_mps2[1];
        accZ = LPF_ACC_ALPHA * accPrev[2] + (1.0f - LPF_ACC_ALPHA) * mpu_data.acc_mps2[2];

        gyrPrev[0] = mpu_data.gyro_rad[0];
        gyrPrev[1] = mpu_data.gyro_rad[1];
        gyrPrev[2] = mpu_data.gyro_rad[2];

        accPrev[0] = mpu_data.acc_mps2[0];
        accPrev[1] = mpu_data.acc_mps2[1];
        accPrev[2] = mpu_data.acc_mps2[2];
    }
};

typedef struct {
	float phi_hat_rad;
	float theta_hat_rad;
}comp;

class CF {
public:
    CF();
    void filter(const MPU6050Data& data, float alpha, float dt_ms);
    AngleEstimate getAngle() const;

private:
    comp state;
};

#ifdef __cplusplus
}
#endif

#endif /* CF_H_ */

