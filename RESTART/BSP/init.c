#include "main.h"

void BSP_Init(void)			//�����ʼ��
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

//���������ʼ������
void ControtLoopTaskInit(void)
{
  time_tick_1ms = 0;   		//�ж��еļ�������
	AppParamInit();    			//���������ʼ��
  shot_param_init();			//��������ʼ��
	gimbal_param_init();		//��̨�����ʼ��
	gun_limit_init();
	RemoteTaskInit();
}

