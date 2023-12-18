#include "Setpoint_Interrupt.h"

void Printf_data(int *number, CLCD_I2C_Name *LCD) {
    char output[4];
    sprintf(output, "%d", *number);
    CLCD_I2C_SetCursor(LCD, 5, 1);
    CLCD_I2C_WriteString(LCD, output);
}

int Data_Down(int *number, BUTTON_Name *BUTTON, CLCD_I2C_Name *LCD, int Min, int Max) {
    (*number) = (Max - Min) / 2; // S?a l?i chính t? ? dây, dã d?i t? (Min - Max) sang (Min + Max)
    BUTTON_STATE state;
    state = BUTTON_Read(BUTTON);
    switch (state) {
        case SINGLE_CLICK:
            (*number) -= 1;
            *number = CLAMP(*number, Min, Max);
            Printf_data(number, LCD);
            break;
        case DOUBLE_CLICK:
            for (int i = 0; i <= 2; i++) {
                (*number) -= 1;
                *number = CLAMP(*number, Min, Max);
                Printf_data(number, LCD);
                HAL_Delay(50);
            }
            break;
        case LONGCLICK_1S:
            for (int i = 0; i <= BUTTON->timePress / 10; i++) {
                (*number) -= 1;
                *number = CLAMP(*number, Min, Max);
                Printf_data(number, LCD);
                HAL_Delay(20);
            }
            break;
        default:
            break;
    }
    return *number;
}

int Data_Up(int *number, BUTTON_Name *BUTTON, CLCD_I2C_Name *LCD, int Min, int Max) {
    (*number) = (Max-Min) / 2; // S?a l?i chính t? ? dây, dã d?i t? (Min - Max) sang (Min + Max)
    BUTTON_STATE state;
    state = BUTTON_Read(BUTTON);
    switch (state) {
        case SINGLE_CLICK:
            (*number) += 1;
            *number = CLAMP(*number, Min, Max);
            Printf_data(number, LCD);
            break;
        case DOUBLE_CLICK:
            for (int i = 0; i <= 2; i++) {
                (*number) += 1;
                *number = CLAMP(*number, Min, Max);
                Printf_data(number, LCD);
                HAL_Delay(50);
            }
            break;
        case LONGCLICK_1S:
            for (int i = 0; i <= BUTTON->timePress / 10; i++) {
                (*number) += 1;
                *number = CLAMP(*number, Min, Max);
                Printf_data(number, LCD);
                HAL_Delay(20);
            }
            break;
        default:
            break;
    }
    return *number;
}

Mode_State Read_Button_Mode(Mode_State currentMode, BUTTON_Name *BUTTON) {
    currentMode = MODE_TIME;
    switch (BUTTON_Read(BUTTON)) {
        case SINGLE_CLICK:
            currentMode++;
            if (currentMode > MODE_TIME) {
                currentMode = MODE_TEMPERATURE;
            }
            break;
        case DOUBLE_CLICK:
            for (int i = 0; i <= 2; i++) {
                currentMode++;
                if (currentMode > MODE_TIME) {
                    currentMode = MODE_TEMPERATURE;
                }
                HAL_Delay(50);
            }
            break;
        case LONGCLICK_1S:
            for (int i = 0; i <= BUTTON->timePress / 10; i++) {
                currentMode++;
                if (currentMode > MODE_TIME) {
                    currentMode = MODE_TEMPERATURE;
                }
                HAL_Delay(20);
            }
            break;
        default:
            break;
    }
    return currentMode;
}

Setpoint Setpoint_Interrupt_Mode(Setpoint *Data, Set_Button *Button, CLCD_I2C_Name *LCD, Mode_State currentMode) {
    switch (Read_Button_Mode(currentMode, &Button->MODE)) {
        case MODE_TIME:
            CLCD_I2C_Clear(LCD);
            CLCD_I2C_SetCursor(LCD, 0, 0);
            CLCD_I2C_WriteString(LCD, "Time(minute):");
            CLCD_I2C_SetCursor(LCD, 5, 1);
            while (Read_Button_Mode(currentMode, &Button->MODE) == MODE_TIME) {
                Data_Down(&Data->time, &Button->DOWN, LCD, 1, 60);
                Data_Up(&Data->time, &Button->UP, LCD, 1, 60);
            }
            break;
        case MODE_HUMIDITY:
            CLCD_I2C_Clear(LCD);
            CLCD_I2C_SetCursor(LCD, 0, 0);
            CLCD_I2C_WriteString(LCD, "Humidity(%):");
            CLCD_I2C_SetCursor(LCD, 5, 1);
            while (Read_Button_Mode(currentMode, &Button->MODE) == MODE_HUMIDITY) {
                Data_Down(&Data->humidity, &Button->DOWN, LCD, 0, 100);
                Data_Up(&Data->humidity, &Button->UP, LCD, 0, 100);
            }
            break;
        case MODE_TEMPERATURE:
            CLCD_I2C_Clear(LCD);
            CLCD_I2C_SetCursor(LCD, 0, 0);
            CLCD_I2C_WriteString(LCD, "Temperature(Celsius):");
            CLCD_I2C_SetCursor(LCD, 5, 1);
            while (Read_Button_Mode(currentMode, &Button->MODE) == MODE_TEMPERATURE) {
                Data_Down(&Data->temperature, &Button->DOWN, LCD, 10, 40);
                Data_Up(&Data->temperature, &Button->UP, LCD, 10, 40);
            }
            break;
        default:
            break;
    }
    return *Data;
}
