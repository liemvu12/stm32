#ifndef __SETPOIN_INTERRUPT_H
#define __SETPOIN_INTERRUPT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "CLCD_I2C.h"
#include "BUTTON.h"
#include "stdio.h"
#include "stdbool.h"

#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

typedef enum {
	MODE_TEMPERATURE , 
	MODE_HUMIDITY ,
	MODE_TIME ,
} Mode_State; 

typedef struct {
	int time;
	int temperature;
	int humidity;
} Setpoint; 

void Printf_Data(int number, char *output);

int Data_Down(int *number, BUTTON_Name *BUTTON, CLCD_I2C_Name *LCD, int Min, int Max);

int Data_Up(int *number, BUTTON_Name *BUTTON, CLCD_I2C_Name *LCD, int Min, int Max);

Mode_State Read_Button_Mode (Mode_State currentMode, BUTTON_Name *BUTTON);

Setpoint Setpoint_Interrupt_Mode(Setpoint Data, BUTTON_Name *Up, BUTTON_Name *Down, CLCD_I2C_Name *LCD,Mode_State currentMode);



#ifdef __cplusplus
}
#endif

#endif

