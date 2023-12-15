#include "Setpoint_Interrupt.h"

void Printf_data(int *number, CLCD_I2C_Name *LCD) {
    char output[4];
    sprintf(output, "%d", *number);
    CLCD_I2C_WriteString(LCD, output);
}

int Humidity_Down(int *number, BUTTON_Name *BUTTON, CLCD_I2C_Name *LCD) {
    BUTTON_STATE state;
    state = BUTTON_Read(BUTTON);
    switch (state) {
        case SINGLE_CLICK:
            (*number) -= 1;
            *number = CLAMP(*number, 20, 100);
            Printf_data(number, LCD); // Replace 'your_LCD_instance' with your actual LCD instance
            break;
        case DOUBLE_CLICK:
            for (int i = 0; i <= 2; i++) {
                (*number) -= 1;
                *number = CLAMP(*number, 20, 100);
                Printf_data(number, LCD);
                HAL_Delay(50);
            }
            break;
        case LONGCLICK_1S:
            for (int i = 0; i <= BUTTON->timePress / 10; i++) {
                (*number) -= 1;
                *number = CLAMP(*number, 20, 100);
                Printf_data(number, LCD);
                HAL_Delay(20);
            }
            break;
        default:
            break;
    }
    return *number;
}

int Humidity_Up(int *number, BUTTON_Name *BUTTON, CLCD_I2C_Name *LCD) {
    BUTTON_STATE state;
    state = BUTTON_Read(BUTTON);
    switch (state) {
        case SINGLE_CLICK:
            (*number) += 1;
            *number = CLAMP(*number, 20, 100);
            Printf_data(number, LCD); // Replace 'your_LCD_instance' with your actual LCD instance
            break;
        case DOUBLE_CLICK:
            for (int i = 0; i <= 2; i++) {
                (*number) += 1;
                *number = CLAMP(*number, 20, 100);
                Printf_data(number, LCD);
                HAL_Delay(50);
            }
            break;
        case LONGCLICK_1S:
            for (int i = 0; i <= BUTTON->timePress / 10; i++) {
                (*number) += 1;
                *number = CLAMP(*number, 20, 100);
                Printf_data(number, LCD);
                HAL_Delay(20);
            }
            break;
        default:
            break;
    }
    return *number;
}
