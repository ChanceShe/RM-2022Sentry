#include "main.h"

void BSP_Init(void)
{
 	Led_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM6_Configuration();
	PWM_Configuration();
//	TIM6_Start(); 
	KEY_Init();
	uart_init(115200);	//串口初始化波特率为115200	
	
	
}


