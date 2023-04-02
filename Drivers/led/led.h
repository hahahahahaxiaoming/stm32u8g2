#ifndef __LED_H
#define __LED_H	 
#include "stm32f10x.h"

#define LED1_GPIO_PORT    	GPIOC			            /* GPIO端口 */
#define LED1_GPIO_CLK 	    RCC_APB2Periph_GPIOC		/* GPIO端口时钟 */
#define LED1_GPIO_PIN		GPIO_Pin_13			        /* 连接到SCL时钟线的GPIO */

#if 0
#define LED1(a)	    if (a)	\
                        GPIO_SetBits(LED1_GPIO_PORT,LED1_GPIO_PIN);\
					else		\
                        GPIO_ResetBits(LED1_GPIO_PORT,LED1_GPIO_PIN)

#define LED1_ON()      GPIO_SetBits(LED1_GPIO_PORT,LED1_GPIO_PIN)
#define LED1_OFF()     GPIO_ResetBits(LED1_GPIO_PORT,LED1_GPIO_PIN)
#else

#define LED1(a)     if (a)\
                        LED1_GPIO_PORT->BSRR = LED1_GPIO_PIN;\
                    else\
                        LED1_GPIO_PORT->BRR = LED1_GPIO_PIN
                        
#define LED1_ON()       LED1_GPIO_PORT->BSRR = LED1_GPIO_PIN
#define LED1_OFF()      LED1_GPIO_PORT->BRR = LED1_GPIO_PIN
#define LED1_BLINK()    LED1_GPIO_PORT->ODR ^= LED1_GPIO_PIN 
#endif

void LED_Init(void);//初始化

#endif
