#include "stm32f10x.h"
#include "./delay/delay.h"
#include "./led/led.h"
#include "./oled/oled.h"
#include "u8g2.h"

int main(void)
{
    Delay_Init();
    LED_Init();
    U8g2_Init();
    
	while(1)
	{
		Delay_Ms(500);
        LED1_BLINK();
	}
}

