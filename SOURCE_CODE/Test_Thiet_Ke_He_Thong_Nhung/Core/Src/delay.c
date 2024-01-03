#include "delay.h"

void SysTick_delay(uint16_t x) {
  SysTick->LOAD = SystemCoreClock / 1000;
  SysTick->VAL = 0;
  SysTick->CTRL = 0x5; // Enable SysTick
for (uint16_t i=0; i< x; i++){
  while ((SysTick->CTRL & (1 << 16)) == 0) {
		}
	}
}
