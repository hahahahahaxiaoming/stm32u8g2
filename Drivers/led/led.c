#include "./led/led.h"
	    
//LED IO��ʼ��
void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(LED1_GPIO_CLK, ENABLE);	        //ʹ��PB�˿�ʱ��

    GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;		    //LED0-->PB.2 �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//IO���ٶ�Ϊ50MHz
    GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);			//�����趨������ʼ��GPIOB.2
    GPIO_ResetBits(LED1_GPIO_PORT,LED1_GPIO_PIN);			//PB.2 �����
}
