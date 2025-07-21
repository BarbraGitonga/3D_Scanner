/*
 * HCSR04.h
 *
 *  Created on: Jul 19, 2025
 *  Author: Barbra Gitonga (barbragitonga@gmail.com)
 */


#ifndef HCSR04_H_
#define HCSR04_H_

#include "stm32f7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    GPIO_TypeDef* trigPort;
    uint16_t trigPin;
    TIM_HandleTypeDef* htim;
    uint32_t timChannel;
    volatile uint32_t ic_rising;
    volatile uint32_t ic_falling;
    volatile uint8_t capture_state;
} HCSR04_HandleTypeDef;

void HCSR04_Init(HCSR04_HandleTypeDef *hcsr04, GPIO_TypeDef* trigPort,
		uint16_t trigPin, TIM_HandleTypeDef* htim, uint32_t timChannel);
void HCSR04_Trigger(HCSR04_HandleTypeDef *hcsr04);
float HCSR04_GetDistance(HCSR04_HandleTypeDef *hcsr04);
void HCSR04_CaptureCallback(HCSR04_HandleTypeDef *hcsr04);

#ifdef __cplusplus
}
#endif

#endif /* HCSR04_H_ */
