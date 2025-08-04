/*
 * MPU9250.cpp
 *
 *  Created on: Jul 31, 2025
 *      Author: Barbra Gitonga(barbragitonga@gmail.com)
 */

#include <MPU9250/MPU9250.h>

MPU9250::MPU9250() {
	// TODO Auto-generated constructor stub

}

HAL_StatusTypeDef MPU9250::writeRegister(MPU9250_data *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write_DMA(dev->i2cHandle, MPU9250_ID, reg, I2C_MEMADD_SIZE_8BIT, data ,1);
}

HAL_StatusTypeDef MPU9250::readRegister(MPU9250_data *dev, uint8_t reg, uint8_t *data){
	return HAL_12C_Mem_Read_DMA(dev->i2cHandle, MPU9250_ID, reg, I2C_MEMADD_SIZE_8BIT, data, 1);
}

HAL_StatusTypeDef MPU9250::burstReadRegister(MPU9250_data *dev, uint8 reg, uint8_t *data, uint8_t length){
	return HAL_I2C_Mem_Read_DMA(dev->i2cHandle, MPU9250_ID, reg, I2C_MEMADD_SIZE_8BIT, data, length);
}

HAL_StatusTypeDef MPU9250::initialize(MPU9250_data *dev, I2C_HandleTypeDef *hi2c){
	/* Setting struct parameters */
	dev->i2cHandle = hi2c;

	dev->acc_mps2[0] = 0.0f;
	dev->acc_mps2[1] = 0.0f;
	dev->acc_mps2[2] = 0.0f;

	dev->gyro_rad[0] = 0.0f;
	dev->gyro_rad[1] = 0.0f;
	dev->gyro_rad[2] = 0.0f;

	dev->temp_C = 0.0f;

	dev->mag_uT[0] = 0.0f;
	dev->mag_uT[1] = 0.0f;
	dev->mag_uT[2] = 0.0f;


	/* Store a number of transaction error */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint8_t regData;

	status = readRegister(dev, WHO_AM_I, &regData);
	errNum += (status != HAL_OK);


	if (regData == 0x75){
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

		HAL_Delay(100);
		return (errNum == 0) ? HAL_OK : HAL_ERROR;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef MPU9250::accelerometer(MPU9250_data *dev) {
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

HAL_StatusTypeDef MPU9250::gyroscope(MPU9250_data *dev){
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

HAL_StatusTypeDef MPU9250::temperature(MPU9250_data *dev){
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

HAL_StatusTypeDef MPU9250::magnetometer(MPU9250_data *dev){
	uint8_t buffer[6];
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	status = burstReadRegister(dev, MAG_HXL, buffer, 6);
	errNum += (status != HAL_OK);

	// resolution is 0.6uT/LSB
	float scale = 0.6f;

	/* Magnetometer values are stored in 2s compliment,and little edian format.*/
	// Combine (int16_t)((MSB << 8) | LSB)
	int16_t mag_x_out = (int16_t)((buffer[1] << 8) | buffer[0]);  // Note: [1] is MSB
	int16_t mag_y_out = (int16_t)((buffer[3] << 8) | buffer[2]);
	int16_t mag_z_out = (int16_t)((buffer[5] << 8) | buffer[4]);

	// convert to uT using the resolution
	dev->mag_uT[0] = mag_x_out * scale;
	dev->mag_uT[1] = mag_x_out * scale;
	dev->mag_uT[2] = mag_x_out * scale;

	return (errNum == 0) ? HAL_OK : HAL_ERROR;

}
MPU9250::~MPU9250() {
	// TODO Auto-generated destructor stub
}

