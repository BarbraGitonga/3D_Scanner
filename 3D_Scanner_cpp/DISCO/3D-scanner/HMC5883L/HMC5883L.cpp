/**
 * @file HMC5883L.cpp
 * @author Barbra Gitonga (barbragitonga@gmail.com)
 * @brief This is the implementation file for the HMC5883L magnetometer driver for the STM32F7 series.
 * It provides functions to initialize the sensor, read magnetometer data, and process the data.
 * The driver uses I2C communication protocol to interact with the sensor.
 * This driver is suitable for polling and DMA-based data acquisition.
 * @version 0.1
 * @date 2025-08-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <HMC5883L/HMC5883L.h>

uint8_t rawBuffer[6]; // stores all raw 8 bit information from the registers

HMC5883L::HMC5883L(){

}

/**
 * @brief Initializes the HMC5883L sensor with the provided I2C handle and data structure.
 * 
 * @param hi2c - Pointer to the I2C handle
 * @param data - Pointer to the HMC_data structure to store sensor data
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef HMC5883L::init(I2C_HandleTypeDef *hi2c, HMC_data *data){
	data->i2c_handle = hi2c;
	data->mag[0] = 0.0f;
	data->mag[1] = 0.0f;
	data->mag[2] = 0.0f;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	// Config A: 8-average, 15 Hz default, normal measurement
	uint8_t config = 0x70;
	status = HAL_I2C_Mem_Write(data->i2c_handle, HMC_ADDR, CONFG_A, I2C_MEMADD_SIZE_8BIT, &config, 1, HAL_MAX_DELAY);
	errNum += (status != HAL_OK);
	if(errNum != 0 ) return HAL_ERROR;

	// Config B: gain = 1090 LSB/Gauss
	config = 0x20;
	status = HAL_I2C_Mem_Write(data->i2c_handle, HMC_ADDR, CONFG_B, I2C_MEMADD_SIZE_8BIT, &config, 1, HAL_MAX_DELAY);
	errNum += (status != HAL_OK);
	if(errNum != 0 ) return HAL_ERROR;

	// Mode: continuous measurement
	config =0x00;
	status = HAL_I2C_Mem_Write(data->i2c_handle, HMC_ADDR, MODE, I2C_MEMADD_SIZE_8BIT, &config, 1, HAL_MAX_DELAY);
	errNum += (status != HAL_OK);
	if(errNum != 0 ) return HAL_ERROR;

	return HAL_OK;
}

/**
 * @brief Reads magnetometer data from the HMC5883L sensor and stores it in the provided data structure.
  The function performs a blocking read operation.
  It reads 6 bytes of data starting from the X-axis MSB register, processes the raw data, and converts it to milliGauss.
  The processed data is stored in the mag array of the HMC_data structure.
  The function returns HAL_OK if the read operation is successful, otherwise returns HAL_ERROR.
  Note: Ensure that the sensor is properly initialized before calling this function.
  This function is suitable for applications where immediate data retrieval is required without using DMA.
 * 
 * @param data - Pointer to the HMC_data structure to store sensor data
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef HMC5883L::read(HMC_data *data){
	uint8_t magBuffer[6]; // stores all raw 8 bit information from the registers

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(data->i2c_handle, HMC_ADDR, X_MAGM, I2C_MEMADD_SIZE_8BIT, magBuffer, 6, HAL_MAX_DELAY);

	if (status != HAL_OK) return HAL_ERROR;

	// processing data. since it is in 2's complement. (MSB << 8 | LSB)
	int16_t raw_x_mag = (int16_t)(magBuffer[0] << 8) | magBuffer[1];
	int16_t raw_z_mag = (int16_t)(magBuffer[2] << 8) | magBuffer[3];
	int16_t raw_y_mag = (int16_t)(magBuffer[4] << 8) | magBuffer[5];

	// Get magnetometer readinsg in milliGauss
	data->mag[0] = raw_x_mag * scale; // x-axis
	data->mag[1] = raw_y_mag * scale; // y-axis
	data->mag[2] = raw_z_mag * scale; // z-axis

	return HAL_OK;
}

/**
 * @brief Reads magnetometer data from the HMC5883L sensor using DMA and stores it in the provided data structure.
 * 
 * @param data - Pointer to the HMC_data structure to store sensor data
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef HMC5883L::readDMA(HMC_data *data){
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read_DMA(data->i2c_handle, HMC_ADDR, X_MAGM, I2C_MEMADD_SIZE_8BIT, rawBuffer, 6);
	if (status != HAL_OK) return status;

	return HAL_OK;
}

/**
 * @brief Convert raw HMC5883L bytes to calibrated magnetic field readings.
 *
 * Interprets six 2's-complement bytes from rawBuffer as MSB-first words
 * in the device order (X, Z, Y), converts to signed 16-bit values, and
 * scales to milliGauss using the scale constant. Stores results in
 * data->mag as [X, Y, Z].
 *
 * Preconditions: rawBuffer holds the latest 6-byte measurement.
 *
 * @param data Pointer to HMC_data that receives the scaled readings; must not be null.
 * @return int8_t 1 on success.
 */
int8_t HMC5883L::data_processing(HMC_data *data){
	// processing data. since it is in 2's complement. (MSB << 8 | LSB)
	int16_t raw_x_mag = (int16_t)(rawBuffer[0] << 8) | rawBuffer[1];
	int16_t raw_z_mag = (int16_t)(rawBuffer[2] << 8) | rawBuffer[3];
	int16_t raw_y_mag = (int16_t)(rawBuffer[4] << 8) | rawBuffer[5];

	// Get magnetometer readinsg in milliGauss
	data->mag[0] = raw_x_mag * scale; // x-axis
	data->mag[1] = raw_y_mag * scale; // y-axis
	data->mag[2] = raw_z_mag * scale; // z-axis

	return 1;
}
