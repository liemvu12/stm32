#include "Setpoint_Interrupt.h"

void Printf_data(int *number, CLCD_Name *LCD) {
    char output[4];
    sprintf(output, "%d", *number);
    CLCD_SetCursor(LCD, 5, 1);
    CLCD_WriteString(LCD, output);
}

int Data_Down(int *number, BUTTON_Name *BUTTON, CLCD_Name *LCD, int Min, int Max) {
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

int Data_Up(int *number, BUTTON_Name *BUTTON, CLCD_Name *LCD, int Min, int Max) {
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

Mode_State Read_Button_Mode(Mode_State currentMode, BUTTON_Name *BUTTON, int *clickCount) {
    if(BUTTON_Read(BUTTON)) {		
		 (*clickCount) ++; 
			}
    switch ((*clickCount) %=3) {
        case 0:
            return MODE_TEMPERATURE;
        case 1:
            return MODE_HUMIDITY;
        case 2:
            return MODE_TIME;
        default:
            return currentMode;
    }
}

Setpoint Setpoint_Interrupt_Mode(Setpoint *Data, Set_Button *Button, CLCD_Name *LCD, Mode_State currentMode,int *clickCount) {
    switch (Read_Button_Mode(currentMode, &Button->MODE, clickCount)) {
        case MODE_HUMIDITY:
            CLCD_Clear(LCD);
            CLCD_SetCursor(LCD, 0, 0);
            CLCD_WriteString(LCD, "Humidity(%):");
            while (1) {
								Printf_data( &Data->humidity, LCD);
                Data_Down(&Data->humidity, &Button->DOWN, LCD, 0, 100);
                Data_Up(&Data->humidity, &Button->UP, LCD, 0, 100);
								if(Read_Button_Mode(currentMode, &Button->MODE, clickCount) != MODE_HUMIDITY)
									break;
            }
            break;
        case MODE_TEMPERATURE:
            CLCD_Clear(LCD);
            CLCD_SetCursor(LCD, 0, 0);
            CLCD_WriteString(LCD, "Temperature(C):");
            while (1) {
                Printf_data( &Data->temperature, LCD);
								Data_Down(&Data->temperature, &Button->DOWN, LCD, 10, 40);
                Data_Up(&Data->temperature, &Button->UP, LCD, 10, 40);
								if(Read_Button_Mode(currentMode, &Button->MODE, clickCount) != MODE_TEMPERATURE)
									break;
            }
            break;
					case MODE_TIME:
            CLCD_Clear(LCD);
            CLCD_SetCursor(LCD, 0, 0);
            CLCD_WriteString(LCD, "Time(minute):");
            while (1) {
								Printf_data( &Data->time, LCD);
								Data_Down(&Data->time, &Button->DOWN, LCD, 1, 60);
                Data_Up(&Data->time, &Button->UP, LCD, 1, 60);
								if(Read_Button_Mode(currentMode, &Button->MODE, clickCount) != MODE_TIME)
									break;
            }
            break;
        default:
            break;
    }
    return *Data;
}
