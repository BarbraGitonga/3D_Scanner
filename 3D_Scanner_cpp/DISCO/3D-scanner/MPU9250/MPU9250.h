/*
 * MPU9250.h
 *
 *  Created on: Jul 31, 2025
 *      Author: lenovo
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174533f

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "string.h"

#define SELF_TEST_X         0x0D
#define SELF_TEST_Y         0x0E
#define SELF_TEST_Z         0x0F
#define SELF_TEST_A         0x10

#define SMPLRT_DIV			0x19
#define CONFIG				0x1A

#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C

//acceleration readings
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40

//temperature readings
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42

//gyroscope readings
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48

//identity
#define WHO_AM_I            0x75 // register that holds identity
#define MPU9250_ID         (0x71 << 1) // default value of who am i register

//power management 1 parameters
#define PWR_MGMT_1          0x6B

//Power management 2
#define PWR_MGMT_2          0x6C

// Interrupt register
#define INT_ENABLE			0x38
#define INT_STATUS			0x3A
#define INT_PIN_CFG			0x37

// FIFO
#define FIFO_EN				0x23

// Magnetometer
#define MAG_HXL				0x03H
#define MAG_HXH				0x04H
#define MAG_HYL 			0x05H
#define MAG_HYH				0x06H
#define MAG_HZL 			0x07H
#define MAG_HZH				0x08H

typedef struct {
	/* I2C HAndle */
	I2C_HandleTypeDef *i2cHandle;

	/* Acceleration data (X, Y, Z) in m/s^2 */
	float acc_mps2[3];

	/* Gyroscope data (X, Y, Z) in rad/s */
	float gyro_rad[3];

	/* Magnetometer data (X, Y, Z) in microtesla*/
	float mag_uT[3];

	/* Temperature data in degrees */
	float temp_C;

} MPU9250_data;

class MPU9250 {
public:
	MPU9250();
	virtual ~MPU9250();
	// Initialization
		HAL_StatusTypeDef initialize(MPU9250_data *dev, I2C_HandleTypeDef *hi2c);

		// Data Acquisition
		HAL_StatusTypeDef readRegister(MPU9250_data *dev, uint8_t reg, uint8_t *data);
		HAL_StatusTypeDef writeRegister(MPU9250_data *dev, uint8_t reg, uint8_t *data);
		HAL_StatusTypeDef burstReadRegister(MPU9250_data *dev, uint8_t reg, uint8_t *data, uint8_t length);

		// Sensor data
		HAL_StatusTypeDef temperature(MPU9250_data *dev);
		HAL_StatusTypeDef gyroscope(MPU9250_data *dev);
		HAL_StatusTypeDef accelerometer(MPU9250_data *dev);
		HAL_StatusTypeDef magnetometer(MPU9250_data *dev);
};

#ifdef __cplusplus
}
#endif

#endif /* MPU9250_H_ */
