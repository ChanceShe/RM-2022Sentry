#include "main.h"

void BSP_Init(void)			//外设初始化
{
	RemoteTaskInit();
	IWDG_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Led_Configuration();
	TIM6_Configuration();
	PWM_Configuration();
	
	USART1_Configuration(100000);
	USART3_Configuration_Send();
	UART4_Configuration();
	BSP_UART5_InitConfig();//接裁判系统

	CAN1_Init();
	CAN2_Init();
	
	KEY_Init();
		
}

//控制任务初始化程序
void ControtLoopTaskInit(void)
{
  time_tick_1ms = 0;   		//中断中的计数清零
  chassis_param_init();		//底盘任务初始化
	
}

