#include "main.h"

void BSP_Init(void)
{
	IWDG_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Led_Configuration();
	TIM6_Configuration();
	PWM_Configuration();
	
	USART1_Configuration(100000);
	USART3_Configuration_Send();
	UART4_Configuration();
	
	CAN1_Init();
	CAN2_Init();
	motorcontrol_init();
	
	KEY_Init();
	
	
}


