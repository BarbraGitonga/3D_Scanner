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

#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174533f

MagCalibration magCalib;

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
HAL_StatusTypeDef HMC5883L::init(I2C_HandleTypeDef *hi2c, HMC_data *data, float declination){
	data->i2c_handle = hi2c;
	data->mag[0] = 0.0f;
	data->mag[1] = 0.0f;
	data->mag[2] = 0.0f;
	data->declination = declination;

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
void HMC5883L::data_processing(HMC_data *data){
	// processing data. since it is in 2's complement. (MSB << 8 | LSB)
	int16_t raw_x_mag = (int16_t)(rawBuffer[0] << 8) | rawBuffer[1];
	int16_t raw_z_mag = (int16_t)(rawBuffer[2] << 8) | rawBuffer[3];
	int16_t raw_y_mag = (int16_t)(rawBuffer[4] << 8) | rawBuffer[5];

// Get magnetometer readings in microTesla
	data->mag[0] = raw_x_mag * scale * 0.1; // x-axis
	data->mag[1] = raw_y_mag * scale * 0.1; // y-axis
	data->mag[2] = raw_z_mag * scale * 0.1; // z-axis

}

/**
 * @brief Calibrates the magnetometer data using provided soft and hard iron calibration parameters.
  The function first processes the raw magnetometer data to obtain the current readings.
  It then applies hard iron calibration by subtracting the hard iron offsets from the raw readings.
  Following this, it applies soft iron calibration using the provided soft iron calibration matrix.
  The calibrated magnetometer readings are stored in the calibrated array of the HMC_data structure.
  The function returns 1 to indicate successful calibration.
  Note: Ensure that the soft_cal matrix and hard_cal array are properly defined and passed to this function for accurate calibration.
  This function is essential for improving the accuracy of magnetometer readings by compensating for distortions caused by nearby magnetic materials.
 * 
 * @param data 
 * @param soft_cal soft iron calibration matrix
 * @param hard_cal hard iron calibration offsets
 * @return int8_t 
 */
int8_t HMC5883L::calibrated_data(HMC_data *data, float (*soft_cal)[3], float *hard_cal){
	float hard[3]; // callibration of hard iron

	data_processing(data);

	// Applying hard iron callibration
	for(uint8_t i = 0; i< 3; i++){
		hard[i] = data->mag[i] - hard_cal[i];
	}

	for(uint8_t i = 0; i < 3; i++){
		data->calibrated[i] = ((soft_cal[i][0] * hard[0]) +
				(soft_cal[i][1] * hard[1])  +
				(soft_cal[i][2] * hard[2])); // 0.1 converts to micorTesla
	}

	return 1;
}

//HAL_StatusTypeDef HMC5883L::callibrate(HMC_data *data, uint8_t samples){
//	int16_t minVal[3] = {32767, 32767, 32767};
//	int16_t maxVal[3] = {-32768, -32768, -32768};
//
//
//	for (uint16_t i = 0; i < samples; i++) {
//		HAL_StatusTypeDef status;
//		status = HAL_I2C_Mem_Read(data->i2c_handle, HMC_ADDR, X_MAGM, I2C_MEMADD_SIZE_8BIT, rawBuffer, 6, HAL_MAX_DELAY);
//		if (status != HAL_OK) return status;
//
//		data_processing(data);
//
//		for (int j = 0; j < 3; j++) {
//			if (raw[j] < minVal[j]) minVal[j] = raw[j];
//			if (raw[j] > maxVal[j]) maxVal[j] = raw[j];
//		}
//
//		HAL_Delay(50); // small delay between samples
//	}
//
//	// Compute hard-iron offset
//	for (int j = 0; j < 3; j++) {
//		magCalib.offset[j] = (maxVal[j] + minVal[j]) / 2.0f;
//	}
//
//	// Compute scale factors (soft-iron approx)
//	float avgDelta = 0;
//	float delta[3];
//	for (int j = 0; j < 3; j++) {
//		delta[j] = (maxVal[j] - minVal[j]) / 2.0f;
//		avgDelta += delta[j];
//	}
//	avgDelta /= 3.0f;
//
//	for (int j = 0; j < 3; j++) {
//		magCalib.scales[j] = avgDelta / delta[j];
//	}
//	return HAL_OK;
//}

/**
 * @brief Calculates the heading (yaw) from the calibrated magnetometer data, compensating for tilt using roll and pitch angles.
  The function first retrieves the calibrated magnetometer readings by applying soft and hard iron calibration.
  It then computes the tilt-compensated magnetic field components in the horizontal plane.
  The heading is calculated using the arctangent of the Y and X components, adjusted for magnetic declination.
  The result is wrapped to ensure it falls within the range of [0, 360) degrees.
  This function is essential for applications requiring accurate compass headings, especially when the device is not level.
  Note: Ensure that roll and pitch angles are provided in radians for correct calculations.
  The function returns the computed heading in degrees.
 * 
 * @param data 
 * @param roll roll angle in degrees
 * @param pitch pitch angle in degrees
 * @param soft_cal soft iron calibration matrix
 * @param hard_cal hard iron calibration offsets
 * @return float 
 */
float HMC5883L::get_heading(HMC_data *data, float roll, float pitch, float (*soft_cal)[3], float *hard_cal){
	float heading;
	// Get calibrated data:
	calibrated_data(data, soft_cal, hard_cal);

	// Converting pitch and roll to radians to use with cos and sin
	float theta = pitch * DEG_TO_RAD;
	float phi = roll * DEG_TO_RAD;

	// roll: phi, pitch: theta
	float ct = cosf(theta); float cp = cosf(phi);
	float st = sinf(theta); float sp = sinf(phi);
	const double pi = 3.14159265358979323846;

	// heading = arctan(hx / hy) but for tilt compensation values of hx and hy:
	float hx = data->calibrated[0] * cp + data->calibrated[1] * sp * st - data->calibrated[2] * ct *  sp; // tilt compensated magnetic x
	float hy = data->calibrated[1] * ct - data->calibrated[2] * st; // tilt compensated magnetic y

	heading = -1 * (atan2(hx,hy) * 180 / pi) + data->declination;

	// Wrap to [0,360)
	if (heading < 0.0f) heading += 360.0f;
	if (heading >= 360.0f) heading -= 360.0f;

	return heading;
}
