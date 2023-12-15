#include "Setpoint_Interrupt.h"
void Printf_Data (int *number, char*output, CLCD_I2C_Name *LCD) {
	fprintf (output, "%d", number);
	CLCD_I2C_WriteString(LCD, output);
}
int Humdity_Down(int *number, BUTTON_Name *BUTTON) {
	BUTTON_STATE state;
	char result[4];
	state = BUTTON_Read(BUTTON);
	swich (state) {
		case SINGLE_CLICK;
			(*number) -=1;
			*number = CLAMP(*number, 20, 100);
			Printf_Data(number, result); 
			break ;
		case DOUBLE_CLICK;
			for (int i =0; i<=2; i++){
				(*number) -=1;
				*number = CLAMP(*number, 20, 100);
				Printf_Data(number, result);
				BUTTON_DelayMs(50);}
			break;
		case LONGCLICK_1S;
			int n =BUTTON->timePress / 10;
			for (int i =0; i<=n; i++){
				(*number) -=1;
				*number = CLAMP(*number, 20, 100);
				Printf_Data(number, result);
				BUTTON_DelayMs(20);}
			break;
		default;
			break;
		}
		return *number; 
}


int Humdity_Up(int *number, BUTTON_Name *BUTTON) {
	BUTTON_STATE state;
	char result[4];
	state = BUTTON_Read(BUTTON);
	swich (state) {
		case SINGLE_CLICK;
			(*number) +=1;
			*number = CLAMP(*number, 20, 100);
			Printf_Data(number, result); 
			break ;
		case DOUBLE_CLICK;
			for (int i =0; i<=2; i++){
				(*number) +=1;
				*number = CLAMP(*number, 20, 100);
				Printf_Data(number, result);
				BUTTON_DelayMs(50);}
			break;
		case LONGCLICK_1S;
			int n =BUTTON->timePress / 10;
			for (int i =0; i<=n; i++){
				(*number) +=1;
				*number = CLAMP(*number, 20, 100);
				Printf_Data(number, result);
				BUTTON_DelayMs(20);}
			break;
		default;
			break;
		}
		return *number; 
}

