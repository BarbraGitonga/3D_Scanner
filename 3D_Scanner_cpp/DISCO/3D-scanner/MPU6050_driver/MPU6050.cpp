/*
 * MPU6050.cpp
 *
 *  Created on: Mar 30, 2025
 *      Author: lenovo
 */

#include <MPU6050_driver/MPU6050.h>
#include <cstdio>
#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174533f

/* LOW-LEVEL FUNCTIONS */
HAL_StatusTypeDef MPU6050::readRegister(MPU6050_data *dev, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_ID, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050::writeRegister(MPU6050_data *dev, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_ID, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050::burstReadRegister(MPU6050_data *dev, uint8_t reg, uint8_t *data, uint8_t length) {
    return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_ID, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}



HAL_StatusTypeDef MPU6050::initialize(MPU6050_data *dev, I2C_HandleTypeDef *hi2c){
	/* Setting struct parameters */
	dev->i2cHandle = hi2c;

	dev->acc_mps2[0] = 0.0f;
	dev->acc_mps2[1] = 0.0f;
	dev->acc_mps2[2] = 0.0f;

	dev->gyro_rad[0] = 0.0f;
	dev->gyro_rad[1] = 0.0f;
	dev->gyro_rad[2] = 0.0f;

	dev->temp_C = 0.0f;

	/* Store a number of transaction error */

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint8_t regData;

	status = readRegister(dev, WHO_AM_I, &regData);
	errNum += (status != HAL_OK);


	if (regData == 0x68){
		// Setting clock speed and enabling sensors
		uint8_t reset = 0x80;
		status = writeRegister(dev, PWR_MGMT_1, &reset);
		errNum += (status != HAL_OK);
		HAL_Delay(100);

		reset = 0x00;
		status = writeRegister(dev, PWR_MGMT_1, &reset);
		errNum += (status != HAL_OK);

		reset = 0x00;
		status = writeRegister(dev, PWR_MGMT_2, &reset);
		errNum += (status != HAL_OK);
		HAL_Delay(100);

		uint8_t rate = 39;
		status = writeRegister(dev, SMPLRT_DIV, &rate);
		errNum += (status != HAL_OK);
		HAL_Delay(50);

		reset = 0x00;
		status = writeRegister(dev, CONFIG, &reset);
		errNum += (status != HAL_OK);
		HAL_Delay(50);

		uint8_t intPinConfig = 0x00;  // Active HIGH
		status = writeRegister(dev, INT_PIN_CFG, &intPinConfig);
		errNum += (status != HAL_OK);
		HAL_Delay(100);

		uint8_t data = 0x01;  // Enable data ready interrupt
		status = writeRegister(dev, INT_ENABLE, &data);
		errNum += (status != HAL_OK);

		uint8_t int_status;
		status = readRegister(dev, INT_STATUS, &int_status);
		errNum += (status != HAL_OK);

		reset = 0x00;
		status = writeRegister(dev, GYRO_CONFIG, &reset); //full scale range of 250
		errNum += (status != HAL_OK);

		reset = 0x00;
		status = writeRegister(dev, ACCEL_CONFIG, &reset); //full scale range of 2g
		errNum += (status != HAL_OK);

		printf("Initialization is complete!");
		HAL_Delay(100);
		return (errNum == 0) ? HAL_OK : HAL_ERROR;
	}
	return HAL_ERROR;

}

HAL_StatusTypeDef MPU6050::accelerometer(MPU6050_data *dev) {
    uint8_t buffer[6];  // Changed from int8_t to uint8_t to match burstReadRegister
    uint8_t errNum = 0;
    HAL_StatusTypeDef status;

    status = burstReadRegister(dev, ACCEL_XOUT_H, buffer, 6);
    errNum += (status != HAL_OK);

    // storing raw data in the buffer variable
    int16_t accel_x_out = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t accel_y_out = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t accel_z_out = (int16_t)((buffer[4] << 8) | buffer[5]);

    float scale = 2.0f / 32768.0f; // convert raw data to m/s^2
    dev->acc_mps2[0] = accel_x_out * scale;
    dev->acc_mps2[1] = accel_y_out * scale;
    dev->acc_mps2[2] = accel_z_out * scale;

    return (errNum == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MPU6050::gyroscope(MPU6050_data *dev){
	uint8_t buffer[6];
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	status = burstReadRegister(dev, GYRO_XOUT_H, buffer, 6);
	errNum += (status != HAL_OK);

	int16_t gyro_x_out = (int16_t)((buffer[0] << 8) | buffer[1]);
	int16_t gyro_y_out = (int16_t)((buffer[2] << 8) | buffer[3]);
	int16_t gyro_z_out = (int16_t)((buffer[4] << 8) | buffer[5]);

	float scale = 250.0 / 32768.0; // convert raw values to degrees per second
	dev->gyro_rad[0] = gyro_x_out * scale * DEG_TO_RAD;
	dev->gyro_rad[1] = gyro_y_out * scale * DEG_TO_RAD;
	dev->gyro_rad[2] = gyro_z_out * scale * DEG_TO_RAD;
	return (errNum == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MPU6050::temperature(MPU6050_data *dev){
	uint8_t buffer[2];
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	status = burstReadRegister(dev, TEMP_OUT_H, buffer, 2);
	errNum += (status != HAL_OK);

	if (errNum == 0) {
        int16_t temp_out = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
        dev->temp_C = (static_cast<float>(temp_out) / 340.0f) + 36.53f;
        return HAL_OK;
    }
    return HAL_ERROR;
}

