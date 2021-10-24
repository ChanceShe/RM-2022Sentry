#include "main.h"

void BSP_Init(void)
{
	IWDG_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Led_Configuration();
	TIM6_Configuration();
	PWM_Configuration();
	USART3_Configuration_Send();
	
	CAN1_Init();
	CAN2_Init();
	motorcontrol_init();
	
	KEY_Init();
	
	
}


