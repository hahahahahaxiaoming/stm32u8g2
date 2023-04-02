#include "./oled/oled.h"
#include "./delay/delay.h"
#include "stm32f10x_spi.h"
#include "u8g2.h"

u8g2_t u8g2;

 #if CONFIG_SW_SPI_U8G2 
void sw_spi_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
    
    /* 使能SPI引脚相关的时钟 */
    OLED_SPI_SCK_APBxClock_FUN (OLED_SPI_SCK_CLK, ENABLE );
    GPIO_InitStructure.GPIO_Pin =  OLED_SPI_SCK_PIN;                   //引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  //频率(50M)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                   //输出类型(推挽式输出)
    GPIO_Init(OLED_SPI_SCK_PORT, &GPIO_InitStructure);

    OLED_SPI_MISO_APBxClock_FUN (OLED_SPI_MISO_CLK, ENABLE );
    GPIO_InitStructure.GPIO_Pin = OLED_SPI_MISO_PIN;                   //输出类型(推挽式输出)
    GPIO_Init(OLED_SPI_MISO_PORT, &GPIO_InitStructure);
    
    OLED_SPI_MOSI_APBxClock_FUN (OLED_SPI_MOSI_CLK, ENABLE );
    GPIO_InitStructure.GPIO_Pin = OLED_SPI_MOSI_PIN;                   //输出类型(推挽式输出)
    GPIO_Init(OLED_SPI_MOSI_PORT, &GPIO_InitStructure);
    
    OLED_SPI_CS_APBxClock_FUN (OLED_SPI_CS_CLK, ENABLE );
    GPIO_InitStructure.GPIO_Pin = OLED_SPI_CS_PIN;                   //输出类型(推挽式输出)
    GPIO_Init(OLED_SPI_CS_PORT, &GPIO_InitStructure);
    GPIO_ResetBits( OLED_SPI_CS_PORT, OLED_SPI_CS_PIN );
    
    OLED_SPI_RES_APBxClock_FUN (OLED_SPI_RES_CLK, ENABLE );
    GPIO_InitStructure.GPIO_Pin = OLED_SPI_RES_PIN;                   //输出类型(推挽式输出)
    GPIO_Init(OLED_SPI_RES_PORT, &GPIO_InitStructure);
    GPIO_SetBits( OLED_SPI_RES_PORT, OLED_SPI_RES_PIN );
    
    OLED_SPI_DC_APBxClock_FUN (OLED_SPI_DC_CLK, ENABLE );
    GPIO_InitStructure.GPIO_Pin = OLED_SPI_DC_PIN;                   //输出类型(推挽式输出)
    GPIO_Init(OLED_SPI_DC_PORT, &GPIO_InitStructure);
 }
 
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
  switch(msg)
  {
    //Initialize SPI peripheral
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
      sw_spi_gpio_init();
      break;
	//Function which implements a delay, arg_int contains the amount of ms
	case U8X8_MSG_DELAY_MILLI:
      Delay_Ms(arg_int);
      break;
	//Function which delays 10us
    case U8X8_MSG_DELAY_10MICRO:
	  Delay_Us(arg_int);
	  break;
	//Function which delays 100ns
//    case U8X8_MSG_DELAY_100NANO:
//      __NOP();
//      break;
    //Function to define the logic level of the clockline
	case U8X8_MSG_GPIO_SPI_CLOCK:
      if(arg_int) GPIO_SetBits(OLED_SPI_SCK_PORT, OLED_SPI_SCK_PIN);
	  else GPIO_ResetBits(OLED_SPI_SCK_PORT, OLED_SPI_SCK_PIN);
      break;
    //Function to define the logic level of the data line to the display
	case U8X8_MSG_GPIO_SPI_DATA:
  	  if(arg_int) GPIO_SetBits(OLED_SPI_MOSI_PORT, OLED_SPI_MOSI_PIN);
      else GPIO_ResetBits(OLED_SPI_MOSI_PORT, OLED_SPI_MOSI_PIN);
	  break;
    // Function to define the logic level of the CS line
	case U8X8_MSG_GPIO_CS:
	  if(arg_int) GPIO_SetBits(OLED_SPI_CS_PORT ,OLED_SPI_CS_PIN);
	  else GPIO_ResetBits(OLED_SPI_CS_PORT, OLED_SPI_CS_PIN);
	  break;
	//Function to define the logic level of the Data/ Command line
	case U8X8_MSG_GPIO_DC:
	  if(arg_int) GPIO_SetBits(OLED_SPI_DC_PORT,OLED_SPI_DC_PIN);
	  else GPIO_ResetBits(OLED_SPI_DC_PORT,OLED_SPI_DC_PIN);
	  break;
	//Function to define the logic level of the RESET line
	case U8X8_MSG_GPIO_RESET:
	  if(arg_int) GPIO_SetBits(OLED_SPI_RES_PORT,OLED_SPI_RES_PIN);
	  else GPIO_ResetBits(OLED_SPI_RES_PORT,OLED_SPI_RES_PIN);
	  break;
	default:
	  return 0; //A message was received which is not implemented, return 0 to indicate an error
	}
	return 1; // command processed successfully.
}

#endif

#if CONFIG_HW_SPI_U8G2 
 void SPI_OLED_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能SPI时钟 */
    OLED_SPI_APBxClock_FUN ( OLED_SPI_CLK, ENABLE );

    /* 使能SPI引脚相关的时钟 */
    OLED_SPI_CS_APBxClock_FUN ( OLED_SPI_CS_CLK|OLED_SPI_SCK_CLK|OLED_SPI_MISO_PIN|OLED_SPI_MOSI_PIN, ENABLE );

//    /* 配置SPI的 CS引脚，普通IO即可 */
//    GPIO_InitStructure.GPIO_Pin = OLED_SPI_CS_PIN;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_Init(OLED_SPI_CS_PORT, &GPIO_InitStructure);
//    GPIO_ResetBits( OLED_SPI_CS_PORT, OLED_SPI_CS_PIN );

    /* 配置SPI的 SCK引脚*/
    GPIO_InitStructure.GPIO_Pin = OLED_SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(OLED_SPI_SCK_PORT, &GPIO_InitStructure);

    /* 配置SPI的 MISO引脚*/
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Pin = OLED_SPI_MISO_PIN;
//    GPIO_Init(OLED_SPI_MISO_PORT, &GPIO_InitStructure);

    /* 配置SPI的 MOSI引脚*/
    GPIO_InitStructure.GPIO_Pin = OLED_SPI_MOSI_PIN;
    GPIO_Init(OLED_SPI_MOSI_PORT, &GPIO_InitStructure);

    OLED_SPI_DC_APBxClock_FUN(OLED_SPI_DC_CLK, ENABLE);	 //使能A端口时钟
    GPIO_InitStructure.GPIO_Pin = OLED_SPI_RES_PIN|OLED_SPI_DC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
    GPIO_Init(OLED_SPI_DC_PORT, &GPIO_InitStructure);	  //初始化GPIOD3,6
    GPIO_SetBits(OLED_SPI_DC_PORT, OLED_SPI_RES_PIN|OLED_SPI_DC_PIN);

    /* SPI 模式配置 */
    // OLED芯片 支持SPI模式0及模式3，据此设置CPOL CPHA
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(OLED_SPIx , &SPI_InitStructure);

    /* 使能 SPI  */
    SPI_Cmd(OLED_SPIx , ENABLE);
	
}

/**
* @brief  使用SPI发送一个字节的数据
* @param  byte：要发送的数据
*/
void SPI_OLED_SendByte(u8 byte)
{
  /* 等待发送缓冲区为空，TXE事件 */
  while (SPI_I2S_GetFlagStatus(OLED_SPIx , SPI_I2S_FLAG_TXE) == RESET);

  /* 写入数据寄存器，把要写入的数据写入发送缓冲区 */
  SPI_I2S_SendData(OLED_SPIx , byte);
}

uint8_t u8x8_byte_3wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    uint8_t *p = (uint8_t*)arg_ptr;
    switch (msg)
    {
        /*通过SPI发送arg_int个字节数据*/
        case U8X8_MSG_BYTE_SEND: 
            
            for(int i = 0; i < arg_int; i++) SPI_OLED_SendByte((u8)*(p + i));

            break;

        /*设置DC引脚，DC引脚控制发送的是数据还是命令*/
        case U8X8_MSG_BYTE_SET_DC: 
            if ( arg_int ) GPIO_SetBits(OLED_SPI_DC_PORT, OLED_SPI_DC_PIN);
            else  GPIO_ResetBits(OLED_SPI_DC_PORT, OLED_SPI_DC_PIN);
            break;
        
        /*初始化函数*/
        case U8X8_MSG_BYTE_INIT: 
            SPI_OLED_Init();
            break;
       
        /* 下面功能无需定义 */
        
        /*开始传输前会进行的操作，如果使用软件片选可以在这里进行控制*/
        case U8X8_MSG_BYTE_START_TRANSFER: 
            break;

        /*传输后进行的操作，如果使用软件片选可以在这里进行控制*/
        case U8X8_MSG_BYTE_END_TRANSFER: 
            break;
        default:
            return 0;
    }
    return 1;
}

uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8,
    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
    U8X8_UNUSED void *arg_ptr) 
{
    return 1;
}
#endif



void U8g2_Init(void)
{
    
#if CONFIG_SW_SPI_U8G2
    u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_4wire_sw_spi, u8g2_gpio_and_delay_stm32);
#endif

#if CONFIG_HW_SPI_U8G2
    u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_3wire_hw_spi, u8g2_gpio_and_delay_stm32);
#endif  
    
    u8g2_InitDisplay(&u8g2);        // 根据所选的芯片进行初始化工作，初始化完成后，显示器处于关闭状态
    u8g2_SetPowerSave(&u8g2, 0);    // 打开显示器

    u8g2_DrawLine(&u8g2, 0, 0, 127, 63); //测试画线
    u8g2_SendBuffer(&u8g2);
}
