#include "Setpoint_Interrupt.h"


static uint16_t mapToRange(uint16_t originalValue, uint16_t newMin, uint16_t newMax) {
    return newMin + ((float)originalValue / 4026.0) * (newMax - newMin);
}

uint16_t Data_Update(uint16_t *number, CLCD_I2C_Name *LCD, uint16_t newMin, uint16_t newMax, ADC_HandleTypeDef *hadc){
	*number = HAL_ADC_GetValue(hadc);
	*number = mapToRange(*number, newMin, newMax);
	char output[5];
  sprintf(output, "%d", *number);
  CLCD_I2C_SetCursor(LCD, 5, 1);
  CLCD_I2C_WriteString(LCD, output);
	return *number ;
}
Mode_State Read_Button_Mode(Mode_State currentMode, BUTTON_Name *BUTTON, uint16_t *clickCount) {
    if(BUTTON_Read(BUTTON) != NO_CLICK) {		
		 (*clickCount) ++; 
			}
    switch ((*clickCount)%3) {
        case 0:
            return MODE_TEMPERATURE;
        case 1:
            return MODE_HUMIDITY;
        case 2:
            return MODE_TIME;
        default:
            break;
    }
		return currentMode;
}

Setpoint Setpoint_Interrupt_Mode(Setpoint *Data, Set_Button *Button, CLCD_I2C_Name *LCD, Mode_State currentMode,uint16_t *clickCount, ADC_HandleTypeDef *hadc) {
	switch (Read_Button_Mode(currentMode, &Button->MODE, clickCount)) {
        case MODE_HUMIDITY:
            CLCD_I2C_Clear(LCD);
            CLCD_I2C_SetCursor(LCD, 0, 0);
            CLCD_I2C_WriteString(LCD, "Temperature(C):");
            while (1) {
								Data_Update(&Data->temperature,LCD, 10, 41, hadc);
								if(Read_Button_Mode(currentMode, &Button->MODE, clickCount) != MODE_TEMPERATURE||BUTTON_Read(&Button->SETTING))
									break;
            }
            break;
        case MODE_TEMPERATURE:
            CLCD_I2C_Clear(LCD);
            CLCD_I2C_SetCursor(LCD, 0, 0);
            CLCD_I2C_WriteString(LCD, "Temperature(C):");
            while (1) {
								Data_Update(&Data->temperature,LCD, 10, 41, hadc);
								if(Read_Button_Mode(currentMode, &Button->MODE, clickCount) != MODE_TEMPERATURE||BUTTON_Read(&Button->SETTING))
									break;
            }
            break;
					case MODE_TIME:
            CLCD_I2C_Clear(LCD);
            CLCD_I2C_SetCursor(LCD, 0, 0);
            CLCD_I2C_WriteString(LCD, "Temperature(C):");
            while (1) {
								Data_Update(&Data->temperature,LCD, 10, 41, hadc);
								if(Read_Button_Mode(currentMode, &Button->MODE, clickCount) != MODE_TEMPERATURE||BUTTON_Read(&Button->SETTING))
									break;
            }
            break;
        default:
            break;
				}
    return *Data;
}