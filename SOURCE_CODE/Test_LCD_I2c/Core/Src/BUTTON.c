#include "button.h"

// Ğ?nh nghia th?i gian nh?n nút d? coi là LONG_CLICK (don v?: millisecond)
#define LONG_PRESS_THRESHOLD 1000
#define DEBOUNCE_DELAY 20 

// Hàm kh?i t?o Button
void BUTTON_Init(BUTTON_Name* Button, GPIO_TypeDef* BUTTON_PORT, uint16_t BUTTON_Pin)
{
    Button->BUTTON_PORT = BUTTON_PORT;
    Button->BUTTON_Pin = BUTTON_Pin;
    Button->State = NO_CLICK;
    Button->timePress = 0;

    // C?u hình chân nút là input_pullup
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
}

// Hàm d?c tr?ng thái nút
BUTTON_STATE BUTTON_Read(BUTTON_Name* Button)
{
    static uint32_t lastPressTime = 0; // Th?i di?m l?n nh?n tru?c dó
    uint32_t currentTime = HAL_GetTick(); // Th?i di?m hi?n t?i

    // Ğ?c giá tr? logic t? chân nút
    uint8_t buttonValue = HAL_GPIO_ReadPin(Button->BUTTON_PORT, Button->BUTTON_Pin);

    // N?u giá tr? logic là 0, có nghia là nút du?c nh?n
    if (buttonValue == GPIO_PIN_RESET)
    {
        // Ki?m tra th?i gian ch?ng nhi?u
        if (currentTime - lastPressTime >= DEBOUNCE_DELAY)
        {
            // N?u tr?ng thái tru?c dó là NO_CLICK
            if (Button->State == NO_CLICK)
            {
                // Ğ?t tr?ng thái là SINGLE_CLICK và b?t d?u d?m th?i gian
                Button->State = SINGLE_CLICK;
                Button->timePress = currentTime;
            }
            else if (Button->State == SINGLE_CLICK)
            {
                // N?u dã nh?n lâu hon LONG_PRESS_THRESHOLD, d?t tr?ng thái là LONG_CLICK
                if ((currentTime - Button->timePress) > LONG_PRESS_THRESHOLD)
                {
                    Button->State = LONG_CLICK;
                }
            }

            // C?p nh?t th?i di?m l?n nh?n tru?c dó
            lastPressTime = currentTime;
        }
    }
    else
    {
        // N?u giá tr? logic là 1, có nghia là nút không du?c nh?n
        if (Button->State != NO_CLICK)
        {
            // N?u tr?ng thái tru?c dó là SINGLE_CLICK ho?c LONG_CLICK, d?t tr?ng thái là NO_CLICK
            Button->State = NO_CLICK;
        }
    }

    return Button->State;
}