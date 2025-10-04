/**
 * @file MPU6050.cpp
 * @author Barbra Gitonga (barbragitonga@gmail.com)
 * @brief This is the header file for the MPU6050 class which provides an
 * interface to interact with the MPU6050 or MPU6000 sensor.
 * @version 0.1
 * @date 2025-07-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef MPU6050_H_
#define MPU6050_H_

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

//temperature readings
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42

//gyroscope readings
#define GYRO_XOUT_H         0x43


//identity
#define WHO_AM_I			0x75
#define MPU6050_ADDR		(0x68 << 1)

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
#include "stm32f7xx_hal.h"
#include "MPU6050.h"

// Scales to convert to respective SI units
#define CONVERT_TO_MPS      2.0f / 32768.0f
#define CONVERT_TO_DEGPS    250.0 / 32768.0

typedef struct {
	/* I2C HAndle */
	I2C_HandleTypeDef *i2cHandle;

	/* Acceleration data (X, Y, Z) in m/s^2 */
	float acc_mps2[3];

	/* Gyroscope data (X, Y, Z) in rad/s */
	float gyro_rad[3];

	/* Magnetometer data (X,  Y, Z) in microtesla*/

	float gyroBias[3];

	float accelBias[3];

	/* Temperature data in degrees */
	float temp_C;

	// flag
	uint8_t rxFlag;
} MPU6050_data;

class MPU6050 {
public:
	MPU6050();
	virtual ~MPU6050();
	// Initialization
		HAL_StatusTypeDef initialize(MPU6050_data *dev, I2C_HandleTypeDef *hi2c);

		// Data Acquisition
		HAL_StatusTypeDef readRegister(MPU6050_data *dev, uint8_t reg, uint8_t *data);
		HAL_StatusTypeDef writeRegister(MPU6050_data *dev, uint8_t reg, uint8_t *data);
		HAL_StatusTypeDef burstReadRegister(MPU6050_data *dev, uint8_t reg, uint8_t *data, uint8_t length);

		HAL_StatusTypeDef read_IMU_DMA(MPU6050_data *dev);
		HAL_StatusTypeDef accelerometer(MPU6050_data *dev);
		HAL_StatusTypeDef gyroscope(MPU6050_data *dev);
		HAL_StatusTypeDef temperature(MPU6050_data *dev);

		// Sensor data
		uint8_t process_data(MPU6050_data *dev);
		HAL_StatusTypeDef callibrate_stationary(MPU6050_data *dev);
};

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H_ */
