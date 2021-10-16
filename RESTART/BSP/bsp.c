#include "main.h"

void BSP_Init(void)
{
 	Led_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM6_Configuration();
	TIM6_Start(); 
	KEY_Init();
	
}


