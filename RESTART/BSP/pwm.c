#include "stm32f4xx.h"
#include "pwm.h"
#include "stm32f4xx.h"

void PWM_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  

	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6, GPIO_AF_TIM3);//????
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_TIM3);
	                        
	  gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;       
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
	  gpio.GPIO_OType = GPIO_OType_PP;
	  gpio.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOC,&gpio);
	
	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6, GPIO_AF_TIM3);  //TIM3->CCR1 ????PWM4
	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_TIM3);  //TIM3->CCR2 ????PWM5
	
	  tim.TIM_Prescaler = 1000-1;    //???
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1680-1;   
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3,&tim);

	  oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
		
    TIM_OC1Init(TIM3,&oc);
   	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);  
	  TIM_OC2Init(TIM3,&oc);
	  TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);  
		
    TIM_ARRPreloadConfig(TIM3,ENABLE);
    TIM_Cmd(TIM3,ENABLE);
		
		PWM1=100,PWM2=100;

}

