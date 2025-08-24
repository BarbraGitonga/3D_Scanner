#ifndef HMC5883L_H_
#define HMC588L_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"

#define HMC_ADDR	(0x1E << 1) // device address

// Device Registers
#define CONFG_A		0x00
#define CONFG_B		0x01
#define MODE		0x02

// data registers
#define X_MAGM		0x03
#define X_MAGL		0x04
#define Z_MAGM		0x05
#define Z_MAGL		0x06
#define Y_MAGM		0x07
#define Y_MAGL		0x08

#define STATUS		0x09

// identity registers
#define ID_A 		0x0A // should return ASCII H (0x48)
#define ID_B		0x0B // should return ASCII 4 (0x04)
#define ID_C		0x0C // should return ASCII 3 (0x03)

// read and write functions
#define scale		0.92

typedef struct {
	I2C_HandleTypeDef *i2c_handle;

	float mag[3];
} HMC_data;

class HMC5883L {
public:
	HMC5883L();
	HAL_StatusTypeDef init(I2C_HandleTypeDef *hi2c, HMC_data *data);
	HAL_StatusTypeDef read(HMC_data *data);
	HAL_StatusTypeDef readDMA(HMC_data *data);

	int8_t data_processing(HMC_data *data);
};

#ifdef __cplusplus
}
#endif

#endif //HMC588L_H_
