#include "Setpoint_Interrupt.h"


static uint16_t mapToRange(uint16_t originalValue, uint16_t newMin, uint16_t newMax) {
    return newMin + ((float)originalValue / 4026.0) * (newMax - newMin);
}
void BUTTON_Setting_Init(BUTTON_Setting* Button, GPIO_TypeDef* BUTTON_PORT, uint16_t BUTTON_Pin)
{
    Button->BUTTON_PORT = BUTTON_PORT;
    Button->BUTTON_Pin = BUTTON_Pin;
}
float Data_Update(CLCD_I2C_Name *LCD, uint16_t newMin, uint16_t newMax, ADC_HandleTypeDef *hadc){
	float number; 
	number = HAL_ADC_GetValue(hadc);
	number = mapToRange(number, newMin, newMax);
	char output[5];
  sprintf(output, "%2.2f ", number);
  CLCD_I2C_SetCursor(LCD, 5, 1);
  CLCD_I2C_WriteString(LCD, output);
	HAL_Delay(100);
	return number ;
}

float Setpoint_Interrupt_Mode( BUTTON_Setting *Button, CLCD_I2C_Name *LCD,ADC_HandleTypeDef *hadc) {
            float adc;
						CLCD_I2C_Clear(LCD);
            CLCD_I2C_SetCursor(LCD, 0, 0);
            CLCD_I2C_WriteString(LCD, "Temperature(C):");
            while (HAL_GPIO_ReadPin(Button->BUTTON_PORT, Button->BUTTON_Pin)) {
							adc = Data_Update(LCD, 10, 41, hadc);
            }
    return adc;
}