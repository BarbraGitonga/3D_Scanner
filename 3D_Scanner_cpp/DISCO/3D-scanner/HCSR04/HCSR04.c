/*
 * HCSR04.c
 *
 *  Created on: Jul 19, 2025
 *      Author: Barbra Gitonga (barbragitonga@gmail.com)
 */

#include "HCSR04.h"


void HCSR04_Init(HCSR04_HandleTypeDef *hcsr04, GPIO_TypeDef* trigPort, uint16_t trigPin,
		TIM_HandleTypeDef *htim, uint32_t timChannel, uint32_t timIT){
	hcsr04->trigPort = trigPort;
	hcsr04->trigPin = trigPin;
	hcsr04->htim = htim;
	hcsr04->timChannel = timChannel;
	hcsr04->ic_rising = 0;
	hcsr04->ic_falling = 0;
	hcsr04->capture_state = 0;
	hcsr04->timIT = timIT;
	hcsr04->distance = 0;
}

void HAL_Delay_us(uint32_t us, HCSR04_HandleTypeDef *hcsr04){
	__HAL_TIM_SET_COUNTER(hcsr04->htim, 0);
	while(__HAL_TIM_GET_COUNTER(hcsr04->htim) < us);
}

// Set the trigger pin high for 10us for 8 cycles
void HCSR04_Trigger(HCSR04_HandleTypeDef *hcsr04){
	HAL_GPIO_WritePin(hcsr04->trigPort, hcsr04->trigPin, GPIO_PIN_SET);
	HAL_Delay_us(10, hcsr04);
	HAL_GPIO_WritePin(hcsr04->trigPort, hcsr04->trigPin, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(hcsr04->htim, hcsr04->timIT);
}

void HCSR04_CaptureCallback(HCSR04_HandleTypeDef *hcsr04) {
	if(hcsr04->capture_state == 0) {
		hcsr04->ic_rising = HAL_TIM_ReadCapturedValue(hcsr04->htim, hcsr04->timChannel);
		hcsr04->capture_state = 1;
		__HAL_TIM_SET_CAPTUREPOLARITY(hcsr04->htim, hcsr04->timChannel,  TIM_INPUTCHANNELPOLARITY_FALLING);
	}

	else if(hcsr04->capture_state == 1){
		hcsr04->ic_falling = HAL_TIM_ReadCapturedValue(hcsr04->htim, hcsr04->timChannel);
		__HAL_TIM_SET_CAPTUREPOLARITY(hcsr04->htim, hcsr04->timChannel, TIM_INPUTCHANNELPOLARITY_RISING);

		__HAL_TIM_SET_COUNTER(hcsr04->htim, 0);

		uint32_t duration;
		if(hcsr04->ic_falling >= hcsr04->ic_rising)
			duration = hcsr04->ic_falling - hcsr04->ic_rising;
		else
			duration = (0XFFFF - hcsr04->ic_rising) + hcsr04->ic_falling;

		__HAL_TIM_SET_CAPTUREPOLARITY(hcsr04->htim, hcsr04->timChannel, TIM_INPUTCHANNELPOLARITY_RISING);
		__HAL_TIM_DISABLE_IT(hcsr04->htim, hcsr04->timIT);

		hcsr04->capture_state = 0;
		hcsr04->distance = duration / 58.0f;
	}
}

float HCSR04_GetDistance(HCSR04_HandleTypeDef *hcsr04){

	HCSR04_Trigger(hcsr04);

	uint32_t t0 = HAL_GetTick();          // ms tick
	while (hcsr04->capture_state < 1) {
		if ((HAL_GetTick() - t0) > 50) {  // 50ms timeout (~>8m range)
			return -1.0f;                 // timeout
		}
	}
	return hcsr04->distance;
}
