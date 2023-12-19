#ifndef __SETPOIN_INTERRUPT_H
#define __SETPOIN_INTERRUPT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "CLCD.h"
#include "BUTTON.h"
#include "stdio.h"
#include "stdbool.h"

#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

typedef enum {
	MODE_TEMPERATURE =0 , 
	MODE_HUMIDITY = 1 ,
	MODE_TIME = 2 ,
} Mode_State; 

typedef struct {
	int time;
	int temperature;
	int humidity;
} Setpoint; 

typedef struct {
	BUTTON_Name UP ;
	BUTTON_Name DOWN ;
	BUTTON_Name MODE;
	BUTTON_Name SETTING;
} Set_Button ;

void Printf_Data(int number, char *output);

int Data_Down(int *number, BUTTON_Name *BUTTON, CLCD_Name *LCD, int Min, int Max);

int Data_Up(int *number, BUTTON_Name *BUTTON, CLCD_Name *LCD, int Min, int Max);

Mode_State Read_Button_Mode (Mode_State currentMode, BUTTON_Name *BUTTON,int *clickCount);

Setpoint Setpoint_Interrupt_Mode(Setpoint *Data, Set_Button *Button, CLCD_Name *LCD, Mode_State currentMode,int *clickCount);


#ifdef __cplusplus
}
#endif

#endif

