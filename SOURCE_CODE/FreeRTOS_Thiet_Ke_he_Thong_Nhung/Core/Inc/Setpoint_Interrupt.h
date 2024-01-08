#ifndef __SETPOIN_INTERRUPT_H
#define __SETPOIN_INTERRUPT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "CLCD_I2C.h"
#include "stdio.h"

#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
typedef struct {
	GPIO_TypeDef* BUTTON_PORT;
	uint16_t BUTTON_Pin;
}BUTTON_Setting;

uint16_t mapToRange(uint16_t originalValue, uint16_t newMin, uint16_t newMax);

void BUTTON_Setting_Init(BUTTON_Setting* Button, GPIO_TypeDef* BUTTON_PORT, uint16_t BUTTON_Pin);

float Data_Update(CLCD_I2C_Name *LCD, uint16_t newMin, uint16_t newMax, ADC_HandleTypeDef *hadc);

float Setpoint_Interrupt_Mode(BUTTON_Setting *Button, CLCD_I2C_Name *LCD,ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif

