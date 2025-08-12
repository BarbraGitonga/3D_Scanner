/*
 * MPU9250.cpp
 *
 *  Created on: Jul 31, 2025
 *      Author: Barbra Gitonga(barbragitonga@gmail.com)
 */

#include <MPU9250/MPU9250.h>

uint8_t imuBuffer[14];
uint8_t magBuffer[6]; // stores all 8 bit information from the registers

MPU9250::MPU9250() {
	// TODO Auto-generated constructor stub

}

HAL_StatusTypeDef MPU9250::writeRegister(MPU9250_data *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev->i2cHandle, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data ,1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU9250::readRegister(MPU9250_data *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
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

	dev->rxFlag = 0;
	/* Store a number of transaction error */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint8_t regData;

	status = readRegister(dev, WHO_AM_I, &regData);
	errNum += (status != HAL_OK);


	if (regData == 0x75 || regData == 0x68){
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

		reset = 0x00;
		status = writeRegister(dev, GYRO_CONFIG, &reset); //full scale range of 250
		errNum += (status != HAL_OK);

		reset = 0x00;
		status = writeRegister(dev, ACCEL_CONFIG, &reset); //full scale range of 2g
		errNum += (status != HAL_OK);

		// Enable bypass mode
		uint8_t bypass = 0x02;
		writeRegister(dev, INT_PIN_CFG, &bypass);
		HAL_Delay(10);


		HAL_Delay(100);
		return (errNum == 0) ? HAL_OK : HAL_ERROR;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef MPU9250::read_MAG_DMA(MPU9250_data *dev){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(
			dev->i2cHandle,
			MPU9250_ADDR,
			MAG_HXL,
			I2C_MEMADD_SIZE_8BIT,
			magBuffer,
			6
		);

	return status;
}

HAL_StatusTypeDef MPU9250::read_IMU_DMA(MPU9250_data *dev){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(dev->i2cHandle, MPU9250_ADDR, ACCEL_XOUT_H,
			I2C_MEMADD_SIZE_8BIT, imuBuffer, 14);

	return status;
}


uint8_t MPU9250::process_data(MPU9250_data *dev){
	int16_t mag_x_out = (int16_t)((magBuffer[1] << 8) | magBuffer[0]);  // Note: [1] is MSB
	int16_t mag_y_out = (int16_t)((magBuffer[3] << 8) | magBuffer[2]);
	int16_t mag_z_out = (int16_t)((magBuffer[5] << 8) | magBuffer[4]);

	int16_t accel_x_out = (int16_t)((imuBuffer[0] << 8) | imuBuffer[1]);
	int16_t accel_y_out = (int16_t)((imuBuffer[2] << 8) | imuBuffer[3]);
	int16_t accel_z_out = (int16_t)((imuBuffer[4] << 8) | imuBuffer[5]);

	int16_t temp_out    = (int16_t)((imuBuffer[6] << 8) | imuBuffer[7]);

	int16_t gyro_x_out  = (int16_t)((imuBuffer[8] << 8) | imuBuffer[9]);
	int16_t gyro_y_out  = (int16_t)((imuBuffer[10] << 8) | imuBuffer[11]);
	int16_t gyro_z_out  = (int16_t)((imuBuffer[12] << 8) | imuBuffer[13]);

	// convert to uT using the resolution
	dev->mag_uT[0] = mag_x_out * CONVERT_TO_UT;
	dev->mag_uT[1] = mag_y_out * CONVERT_TO_UT;
	dev->mag_uT[2] = mag_z_out * CONVERT_TO_UT;

	// convert readings to mps^2
	dev->acc_mps2[0] = accel_x_out * CONVERT_TO_MPS;
	dev->acc_mps2[1] = accel_y_out * CONVERT_TO_MPS;
	dev->acc_mps2[2] = accel_z_out * CONVERT_TO_MPS;

	// convert to degrees celcius
	dev->temp_C = (static_cast<float>(temp_out) / 340.0f) + 36.53f;

	// convert to radians per second
	dev->gyro_rad[0] = gyro_x_out * CONVERT_TO_DEGPS * DEG_TO_RAD;
	dev->gyro_rad[1] = gyro_y_out * CONVERT_TO_DEGPS * DEG_TO_RAD;
	dev->gyro_rad[2] = gyro_z_out * CONVERT_TO_DEGPS * DEG_TO_RAD;

	return dev->rxFlag = 1;
}

MPU9250::~MPU9250() {
	// TODO Auto-generated destructor stub
}

