#ifndef __SETPOIN_INTERRUPT_H
#define __SETPOIN_INTERRUPT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "CLCD_I2C.h"
#include "BUTTON.h"
#include "stdint.h"

#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

void Printf_Data(int number, char *output);
int Humdity_Down(int *number, BUTTON_Name *BUTTON);


int Humdity_Up(int *number, BUTTON_Name *BUTTON);

#ifdef __cplusplus
}
#endif

#endif

