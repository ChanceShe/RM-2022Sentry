#include "main.h"

/*----GREEN LED----PC1-----'0' is on,'1' is off */
/*----RED LED----PC2-----'0' is on,'1' is off */

void Led_Configuration(void)
{
  GPIO_InitTypeDef gpio;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);
   

	
	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&gpio);
	
	
	
	GREEN_LED_OFF();
	RED_LED_OFF();
	gpio.GPIO_Pin = GPIO_Pin_4;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&gpio);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI3);
}

void Laser_Configuration()
{
    GPIO_InitTypeDef gpioInitStruct;
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE );
    gpioInitStruct.GPIO_Pin = GPIO_Pin_8;
    gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
    gpioInitStruct.GPIO_OType = GPIO_OType_PP;
    gpioInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init ( GPIOA, &gpioInitStruct );
    GPIO_ResetBits ( GPIOA, GPIO_Pin_8 );
}

