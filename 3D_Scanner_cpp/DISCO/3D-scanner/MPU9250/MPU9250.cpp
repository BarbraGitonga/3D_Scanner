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

HAL_StatusTypeDef MPU9250::burstReadRegister(MPU9250_data *dev, uint8_t reg, uint8_t *data, uint8_t length) {
    return HAL_I2C_Mem_Read(dev->i2cHandle, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU9250::initialize(MPU9250_data *dev, I2C_HandleTypeDef *hi2c) {
    /* Store handle */
    dev->i2cHandle = hi2c;

    /* Zero all data arrays */
    for (int i = 0; i < 3; i++) {
        dev->acc_mps2[i] = 0.0f;
        dev->gyro_rad[i] = 0.0f;
        dev->mag_uT[i] = 0.0f;
        dev->accelBias[i] = 0.0f;
        dev->gyroBias[i] = 0.0f;
        dev->magBias[i] = 0.0f;
    }
    dev->rxFlag = 0;

    uint8_t regData;
    uint8_t errNum = 0;
    HAL_StatusTypeDef status;

    /* WHO_AM_I check */
    status = readRegister(dev, WHO_AM_I, &regData);
    errNum += (status != HAL_OK);

    if (regData == 0x75) {   // keeping your original identity check
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

        /* --- Enable bypass mode properly --- */
        uint8_t userCtrl = 0x00; // Disable I2C master
        writeRegister(dev, USER_CTRL, &userCtrl);
        HAL_Delay(10);

        uint8_t bypass = 0x02; // I2C_BYPASS_EN
        writeRegister(dev, INT_PIN_CFG, &bypass);
        HAL_Delay(10);

        /* --- Configure AK8963 magnetometer --- */
        uint8_t magCtrl;

        // Power down magnetometer
        magCtrl = 0x00;
        HAL_I2C_Mem_Write(dev->i2cHandle, AKM_ID, CNTRL_1, I2C_MEMADD_SIZE_8BIT, &magCtrl, 1, HAL_MAX_DELAY);
        HAL_Delay(10);

        // Enter fuse ROM access mode (optional for calibration)
        magCtrl = 0x0F;
        HAL_I2C_Mem_Write(dev->i2cHandle, AKM_ID, 0x0A, I2C_MEMADD_SIZE_8BIT, &magCtrl, 1, HAL_MAX_DELAY);
        HAL_Delay(10);

        // Read adjustment values here if needed...

        // Power down again
        magCtrl = 0x00;
        HAL_I2C_Mem_Write(dev->i2cHandle, AKM_ID, 0x0A, I2C_MEMADD_SIZE_8BIT, &magCtrl, 1, HAL_MAX_DELAY);
        HAL_Delay(10);

        // Continuous measurement mode 2 (100Hz), 16-bit
        magCtrl = 0x16;
        HAL_I2C_Mem_Write(dev->i2cHandle, AKM_ID, 0x0A, I2C_MEMADD_SIZE_8BIT, &magCtrl, 1, HAL_MAX_DELAY);
        HAL_Delay(10);

        return (errNum == 0) ? HAL_OK : HAL_ERROR;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef MPU9250::read_MAG_DMA(MPU9250_data *dev){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(
			dev->i2cHandle,
			AKM_ID,
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
	dev->mag_uT[0] = (mag_x_out * CONVERT_TO_UT) - dev->magBias[0];
	dev->mag_uT[1] = (mag_y_out * CONVERT_TO_UT) - dev->magBias[1];
	dev->mag_uT[2] = (mag_z_out * CONVERT_TO_UT) - dev->magBias[2];

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
	uint8_t Buffer[6];
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	status = burstReadRegister(dev, MAG_HXL, Buffer, 6);
	errNum += (status != HAL_OK);

	int16_t mag_x_out = (int16_t)((Buffer[1] << 8) | Buffer[0]);  // Note: [1] is MSB
	int16_t mag_y_out = (int16_t)((Buffer[3] << 8) | Buffer[2]);
	int16_t mag_z_out = (int16_t)((Buffer[5] << 8) | Buffer[4]);

	// convert to uT using the resolution
	dev->mag_uT[0] = (mag_x_out * CONVERT_TO_UT) - dev->magBias[0];
	dev->mag_uT[1] = (mag_y_out * CONVERT_TO_UT) - dev->magBias[1];
	dev->mag_uT[2] = (mag_z_out * CONVERT_TO_UT) - dev->magBias[2];

	return (errNum == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MPU9250::callibrate_stationary(MPU9250_data *data){
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

HAL_StatusTypeDef MPU9250::callibrate_mag(MPU9250_data *data){
   // ----------- Magnetometer Calibration (Figure-8 motion) -----------
	// Collect max/min for each axis
	float mxMin = 32767, myMin = 32767, mzMin = 32767;
	float mxMax = -32768, myMax = -32768, mzMax = -32768;

	uint32_t startTime = HAL_GetTick();
	while (HAL_GetTick() - startTime < 15000) { // 15 seconds
		if (magnetometer(data) != HAL_OK) {
			return HAL_ERROR;
		}

		if (data->mag_uT[0] < mxMin) mxMin = data->mag_uT[0];
		if (data->mag_uT[1] < myMin) myMin = data->mag_uT[1];
		if (data->mag_uT[2] < mzMin) mzMin = data->mag_uT[2];

		if (data->mag_uT[0] > mxMax) mxMax = data->mag_uT[0];
		if (data->mag_uT[1] > myMax) myMax = data->mag_uT[1];
		if (data->mag_uT[2] > mzMax) mzMax = data->mag_uT[2];

		HAL_Delay(12); // ~80 Hz
	}

	data->magBias[0] = (mxMax + mxMin) / 2.0f;
	data->magBias[1] = (myMax + myMin) / 2.0f;
	data->magBias[2] = (mzMax + mzMin) / 2.0f;

	return HAL_OK;
}

MPU9250::~MPU9250() {
	// TODO Auto-generated destructor stub
}

