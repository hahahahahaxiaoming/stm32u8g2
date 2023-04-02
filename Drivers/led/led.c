#include "./led/led.h"
	    
//LED IO初始化
void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(LED1_GPIO_CLK, ENABLE);	        //使能PB端口时钟

    GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;		    //LED0-->PB.2 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//IO口速度为50MHz
    GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);			//根据设定参数初始化GPIOB.2
    GPIO_ResetBits(LED1_GPIO_PORT,LED1_GPIO_PIN);			//PB.2 输出低
}
