#ifndef __OLED_H
#define __OLED_H 

#include "stm32f10x.h"


/* ֻ��ѡ��һ����ע��˿ں� */
#define CONFIG_SW_SPI_U8G2 0
#define CONFIG_HW_SPI_U8G2 1



//SPIѡ��
#define      OLED_SPIx                        SPI1
#define      OLED_SPI_APBxClock_FUN          RCC_APB2PeriphClockCmd
#define      OLED_SPI_CLK                     RCC_APB2Periph_SPI1

//SCK����
#define      OLED_SPI_SCK_APBxClock_FUN      RCC_APB2PeriphClockCmd
#define      OLED_SPI_SCK_CLK                 RCC_APB2Periph_GPIOA   
#define      OLED_SPI_SCK_PORT                GPIOA   
#define      OLED_SPI_SCK_PIN                 GPIO_Pin_5
//MISO����
#define      OLED_SPI_MISO_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define      OLED_SPI_MISO_CLK                RCC_APB2Periph_GPIOA    
#define      OLED_SPI_MISO_PORT               GPIOA 
#define      OLED_SPI_MISO_PIN                GPIO_Pin_6
//MOSI����
#define      OLED_SPI_MOSI_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define      OLED_SPI_MOSI_CLK                RCC_APB2Periph_GPIOA    
#define      OLED_SPI_MOSI_PORT               GPIOA 
#define      OLED_SPI_MOSI_PIN                GPIO_Pin_7

//CS(NSS)���� Ƭѡѡ��ͨGPIO����//�������ͣ���������
#define      OLED_SPI_CS_APBxClock_FUN        RCC_APB2PeriphClockCmd
#define      OLED_SPI_CS_CLK                  RCC_APB2Periph_GPIOA  
#define      OLED_SPI_CS_PORT                 GPIOA
#define      OLED_SPI_CS_PIN                  GPIO_Pin_4

//RES���� Ƭѡѡ��ͨGPIO����//���Խӵ�ϵͳ��λ���Ż��߸ߵ�ƽ
#define      OLED_SPI_RES_APBxClock_FUN       RCC_APB2PeriphClockCmd
#define      OLED_SPI_RES_CLK                  RCC_APB2Periph_GPIOB  
#define      OLED_SPI_RES_PORT                 GPIOB
#define      OLED_SPI_RES_PIN                  GPIO_Pin_0

//DC���� Ƭѡѡ��ͨGPIO����
#define      OLED_SPI_DC_APBxClock_FUN       RCC_APB2PeriphClockCmd
#define      OLED_SPI_DC_CLK                  RCC_APB2Periph_GPIOB  
#define      OLED_SPI_DC_PORT                 GPIOB
#define      OLED_SPI_DC_PIN                  GPIO_Pin_1

//#define  		OLED_CS_LOW()     						GPIO_ResetBits( OLED_SPI_CS_PORT, OLED_SPI_CS_PIN )
//#define  		OLED_CS_HIGH()    						GPIO_SetBits( OLED_SPI_CS_PORT, OLED_SPI_CS_PIN )

//#define         OLED_RST_Clr()                          GPIO_ResetBits(OLED_SPI_RES_PORT, OLED_SPI_DC_PIN)//RES
//#define         OLED_RST_Set()                          GPIO_SetBits(OLED_SPI_RES_PORT, OLED_SPI_DC_PIN)

//#define         OLED_DC_Clr()                           GPIO_ResetBits(OLED_SPI_DC_PORT, OLED_SPI_DC_PIN)//DC
//#define         OLED_DC_Set()                           GPIO_SetBits(OLED_SPI_DC_PORT, OLED_SPI_DC_PIN)


void U8g2_Init(void);

#endif
