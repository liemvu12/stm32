
#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f1xx_hal.h"

typedef enum
{
	NO_CLICK, 		
  SINGLE_CLICK ,
	LONG_CLICK ,
}BUTTON_STATE;

typedef struct {
	GPIO_TypeDef* BUTTON_PORT;
	uint16_t BUTTON_Pin;
	BUTTON_STATE State;
	uint16_t timePress;
	uint16_t isTime;
}BUTTON_Name;

void BUTTON_Init(BUTTON_Name* Button, GPIO_TypeDef* BUTTON_PORT, uint16_t BUTTON_Pin);
BUTTON_STATE BUTTON_Read(BUTTON_Name* Button);

#endif

