/**
 * @file MPU6050.cpp
 * @author Barbra Gitonga (barbragitonga@gmail.com)
 * @brief This file contains the implementation of the MPU6050 class for interfacing with the MPU6050 and MPU6000 sensor.
  It includes methods for initializing the sensor, reading data, and processing the data.
  The class uses I2C communication to interact with the sensor.
 * @version 0.1
 * @date 2025-07-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <MPU6050/MPU6050.h>

uint8_t imuBuffer[14];
uint8_t magBuffer[6]; // stores all 8 bit information from the registers

/**
 * @brief Construct a new MPU6050::MPU6050 object
 * 
 */
MPU6050::MPU6050() {
	// TODO Auto-generated constructor stub

}

/**
 * @brief Write to a register
 * 
 * @param dev -  a pointer to the MPU6050_data structure
 * @param reg - register address
 * @param data - pointer to the data to be written
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef MPU6050::writeRegister(MPU6050_data *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data ,1, HAL_MAX_DELAY);
}

/**
 * @brief Read from a register
 * 
 * @param dev - a pointer to the MPU6050_data structure
 * @param reg - register address
 * @param data - pointer to the data to be read
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef MPU6050::readRegister(MPU6050_data *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Burst read from multiple registers
 * 
 * @param dev - a pointer to the MPU6050_data structure
 * @param reg - starting register address
 * @param data - pointer to the data buffer to store read values
 * @param length - number of bytes to read
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef MPU6050::burstReadRegister(MPU6050_data *dev, uint8_t reg, uint8_t *data, uint8_t length) {
    return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

/**
 * @brief Initialize the MPU6050 sensor
 * 
 * @param dev a pointer to the MPU6050_data structure
 * @param hi2c a pointer to the I2C handle
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef MPU6050::initialize(MPU6050_data *dev, I2C_HandleTypeDef *hi2c) {
    /* Store handle */
    dev->i2cHandle = hi2c;

    /* Zero all data arrays */
    for (int i = 0; i < 3; i++) {
        dev->acc_mps2[i] = 0.0f;
        dev->gyro_rad[i] = 0.0f;
        dev->accelBias[i] = 0.0f;
        dev->gyroBias[i] = 0.0f;
    }
    dev->rxFlag = 0;

    uint8_t regData;
    uint8_t errNum = 0;
    HAL_StatusTypeDef status;

    /* WHO_AM_I check */
    status = readRegister(dev, WHO_AM_I, &regData);
    errNum += (status != HAL_OK);

    if (regData == 0x68) {   // keeping your original identity check
        /* Reset device */
        uint8_t reset = 0x80;
        status = writeRegister(dev, PWR_MGMT_1, &reset);
        errNum += (status != HAL_OK);
        HAL_Delay(100);

        reset = 0x00; // Clock source
        status = writeRegister(dev, PWR_MGMT_1, &reset);
        errNum += (status != HAL_OK);

        reset = 0x00; // Enable all sensors
        status = writeRegister(dev, PWR_MGMT_2, &reset);
        errNum += (status != HAL_OK);
        HAL_Delay(100);

        /* Sample rate and config */
        uint8_t rate = 39;
        status = writeRegister(dev, SMPLRT_DIV, &rate);
        errNum += (status != HAL_OK);

        reset = 0x00; // DLPF config
        status = writeRegister(dev, CONFIG, &reset);
        errNum += (status != HAL_OK);

        reset = 0x00; // Gyro ±250dps
        status = writeRegister(dev, GYRO_CONFIG, &reset);
        errNum += (status != HAL_OK);

        reset = 0x00; // Accel ±2g
        status = writeRegister(dev, ACCEL_CONFIG, &reset);
        errNum += (status != HAL_OK);


        return (errNum == 0) ? HAL_OK : HAL_ERROR;
    }
    return HAL_ERROR;
}

/**
 * @brief Read IMU data using DMA
 * 
 * @param dev - a pointer to the MPU6050_data structure
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef MPU6050::read_IMU_DMA(MPU6050_data *dev){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(dev->i2cHandle, MPU6050_ADDR, ACCEL_XOUT_H,
			I2C_MEMADD_SIZE_8BIT, imuBuffer, 14);

	return status;
}

/**
 * @brief Process raw IMU data and convert to physical units
 * 
 * @param dev - a pointer to the MPU6050_data structure 
 * @return uint8_t 
 */
uint8_t MPU6050::process_data(MPU6050_data *dev){

	int16_t accel_x_out = (int16_t)((imuBuffer[0] << 8) | imuBuffer[1]);
	int16_t accel_y_out = (int16_t)((imuBuffer[2] << 8) | imuBuffer[3]);
	int16_t accel_z_out = (int16_t)((imuBuffer[4] << 8) | imuBuffer[5]);

	int16_t temp_out    = (int16_t)((imuBuffer[6] << 8) | imuBuffer[7]);

	int16_t gyro_x_out  = (int16_t)((imuBuffer[8] << 8) | imuBuffer[9]);
	int16_t gyro_y_out  = (int16_t)((imuBuffer[10] << 8) | imuBuffer[11]);
	int16_t gyro_z_out  = (int16_t)((imuBuffer[12] << 8) | imuBuffer[13]);


	// convert readings to mps^2
	dev->acc_mps2[0] = (accel_x_out * CONVERT_TO_MPS) - dev->accelBias[0];
	dev->acc_mps2[1] = (accel_y_out * CONVERT_TO_MPS) - dev->accelBias[1];
	dev->acc_mps2[2] = (accel_z_out * CONVERT_TO_MPS) - dev->accelBias[2];

	// convert to degrees celcius
	dev->temp_C = (static_cast<float>(temp_out) / 340.0f) + 36.53f;

	// convert to radians per second
	dev->gyro_rad[0] = (gyro_x_out * CONVERT_TO_DEGPS * DEG_TO_RAD) - dev->gyroBias[0];
	dev->gyro_rad[1] = (gyro_y_out * CONVERT_TO_DEGPS * DEG_TO_RAD) - dev->gyroBias[1];
	dev->gyro_rad[2] = (gyro_z_out * CONVERT_TO_DEGPS * DEG_TO_RAD) - dev->gyroBias[2];

	return dev->rxFlag = 1;
}

/**
 * @brief Read accelerometer data
 * 
 * @param dev -  a pointer to the MPU6050_data structure
 * @return HAL_StatusTypeDef 
 */
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

/**
 * @brief Read gyroscope data
 * 
 * @param dev -  a pointer to the MPU6050_data structure
 * @return HAL_StatusTypeDef 
 */
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

/**
 * @brief Read temperature data
 * 
 * @param dev - a pointer to the MPU6050_data structure
 * @return HAL_StatusTypeDef 
 */
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

/**
 * @brief Calibrate the sensor while stationary to determine biases
 * 
 * @param data - a pointer to the MPU6050_data structure
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef MPU6050::callibrate_stationary(MPU6050_data *data){
    const uint16_t samples = 2000;   // More samples = better accuracy
    const uint16_t delayMs = 2;      // Delay between samples

    float gxSum = 0, gySum = 0, gzSum = 0;
    float axSum = 0, aySum = 0, azSum = 0;

    // ----------- Gyro & Accel Calibration (Stationary) -----------
    for (uint16_t i = 0; i < samples; i++) {
        if (accelerometer(data) != HAL_OK || gyroscope(data) != HAL_OK) {
            return HAL_ERROR;
        }

        gxSum += data->gyro_rad[0];
        gySum += data->gyro_rad[1];
        gzSum += data->gyro_rad[2];

        axSum += data->acc_mps2[0];
        aySum += data->acc_mps2[1];
        azSum += data->acc_mps2[2];

        HAL_Delay(delayMs);
    }

    data->gyroBias[0] = gxSum / samples;
    data->gyroBias[1] = gySum / samples;
    data->gyroBias[2] = gzSum / samples;

    // For accel, subtract gravity from Z-axis (assuming Z faces up)
    data->accelBias[0] = axSum / samples;
    data->accelBias[1] = aySum / samples;
    data->accelBias[2] = (azSum / samples) - 1.0f; // 1g gravity

    return HAL_OK;
}


MPU6050::~MPU6050() {
	// TODO Auto-generated destructor stub
}

