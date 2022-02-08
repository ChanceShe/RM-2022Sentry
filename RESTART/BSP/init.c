#include "main.h"

void BSP_Init(void)			//�����ʼ��
{
	RemoteTaskInit();
	IWDG_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Led_Configuration();
	Laser_Configuration();
	TIM6_Configuration();
	PWM_Configuration();
	
	USART1_Configuration(100000);
	USART3_Configuration_Send();
	UART4_Configuration();
  USART6_Configuration_For_Hi220();
	
	CAN1_Init();
	CAN2_Init();
	
	KEY_Init(); 
		
}

//���������ʼ������
void ControtLoopTaskInit(void)
{
  time_tick_1ms = 0;   		//�ж��еļ�������
	AppParamInit();    			//���������ʼ��
  chassis_param_init();		//���������ʼ��
  shot_param_init();			//��������ʼ��
	gimbal_param_init();		//��̨�����ʼ��
}

