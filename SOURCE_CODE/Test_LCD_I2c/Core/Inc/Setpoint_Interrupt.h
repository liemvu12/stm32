#ifndef __SETPOIN_INTERRUPT_H
#define __SETPOIN_INTERRUPT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "CLCD_I2C.h"
#include "BUTTON.h"
#include "stdio.h"

#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
static volatile uint8_t interruptFlag = 0;

typedef enum {
	MODE_TEMPERATURE =0 , 
	MODE_HUMIDITY = 1 ,
	MODE_TIME = 2 ,
} Mode_State; 

typedef struct {
	uint16_t time;
	uint16_t temperature;
	uint16_t humidity;
} Setpoint; 

typedef struct {
	BUTTON_Name MODE;
	BUTTON_Name SETTING;
} Set_Button ;

uint16_t Data_Update(uint16_t *number, CLCD_I2C_Name *LCD, uint16_t newMin, uint16_t newMax, ADC_HandleTypeDef *hadc);

Mode_State Read_Button_Mode (Mode_State currentMode, BUTTON_Name *BUTTON,uint16_t *clickCount);

Setpoint Setpoint_Interrupt_Mode(Setpoint *Data, Set_Button *Button, CLCD_I2C_Name *LCD, Mode_State currentMode,uint16_t *clickCount, ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif

