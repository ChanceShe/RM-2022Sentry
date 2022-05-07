#include "main.h"

void BSP_Init(void)			//外设初始化
{
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Led_Configuration();
	Laser_Configuration();
	PWM_Configuration();
	KEY_Init(); 
	
	TIM6_Configuration();
	
	USART3_Configuration_Send();
	UART4_Configuration();
  USART6_Configuration_For_CH100();

	delay_ms (100);
	
	
	CAN1_Init();
	CAN2_Init();

	


	IWDG_Configuration();
	
}

//控制任务初始化程序
void ControtLoopTaskInit(void)
{
  time_tick_1ms = 0;   		//中断中的计数清零
	AppParamInit();    			//程序参数初始化
  shot_param_init();			//射击任务初始化
	gimbal_param_init();		//云台任务初始化
	gun_limit_init();
	RemoteTaskInit();
}

