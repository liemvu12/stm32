#include "button.h"

// �?nh nghia th?i gian nh?n n�t d? coi l� LONG_CLICK (don v?: millisecond)
#define LONG_PRESS_THRESHOLD 1000
#define DEBOUNCE_DELAY 20 

// H�m kh?i t?o Button
void BUTTON_Init(BUTTON_Name* Button, GPIO_TypeDef* BUTTON_PORT, uint16_t BUTTON_Pin)
{
    Button->BUTTON_PORT = BUTTON_PORT;
    Button->BUTTON_Pin = BUTTON_Pin;
    Button->State = NO_CLICK;
    Button->timePress = 0;

    // C?u h�nh ch�n n�t l� input_pullup
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
}

// H�m d?c tr?ng th�i n�t
BUTTON_STATE BUTTON_Read(BUTTON_Name* Button)
{
    static uint32_t lastPressTime = 0; // Th?i di?m l?n nh?n tru?c d�
    uint32_t currentTime = HAL_GetTick(); // Th?i di?m hi?n t?i

    // �?c gi� tr? logic t? ch�n n�t
    uint8_t buttonValue = HAL_GPIO_ReadPin(Button->BUTTON_PORT, Button->BUTTON_Pin);

    // N?u gi� tr? logic l� 0, c� nghia l� n�t du?c nh?n
    if (buttonValue == GPIO_PIN_RESET)
    {
        // Ki?m tra th?i gian ch?ng nhi?u
        if (currentTime - lastPressTime >= DEBOUNCE_DELAY)
        {
            // N?u tr?ng th�i tru?c d� l� NO_CLICK
            if (Button->State == NO_CLICK)
            {
                // �?t tr?ng th�i l� SINGLE_CLICK v� b?t d?u d?m th?i gian
                Button->State = SINGLE_CLICK;
                Button->timePress = currentTime;
            }
            else if (Button->State == SINGLE_CLICK)
            {
                // N?u d� nh?n l�u hon LONG_PRESS_THRESHOLD, d?t tr?ng th�i l� LONG_CLICK
                if ((currentTime - Button->timePress) > LONG_PRESS_THRESHOLD)
                {
                    Button->State = LONG_CLICK;
                }
            }

            // C?p nh?t th?i di?m l?n nh?n tru?c d�
            lastPressTime = currentTime;
        }
    }
    else
    {
        // N?u gi� tr? logic l� 1, c� nghia l� n�t kh�ng du?c nh?n
        if (Button->State != NO_CLICK)
        {
            // N?u tr?ng th�i tru?c d� l� SINGLE_CLICK ho?c LONG_CLICK, d?t tr?ng th�i l� NO_CLICK
            Button->State = NO_CLICK;
        }
    }

    return Button->State;
}