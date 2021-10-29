#include "main.h"

volatile Encoder CM11Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM21Encoder = {0,0,0,0,0,0,0,0,0};

void motorcontrol_init(void)
{
	PID_struct_init(&pid_motor1 , POSITION_PID , 5000 , 5000 , 50 ,0 ,40);
	PID_struct_init(&pid_motor2 , POSITION_PID , 5000 , 5000 , 50 ,0 ,40);
}

//int motorcontrol_flag = 1;

void moter_control(void)//电机控制任务
{	
	
	pid_motor1.get=CM11Encoder.filter_rate;//CAN1电机
  pid_motor2.get=CM21Encoder.filter_rate;//CAN2电机		
	
	pid_calc(&pid_motor1, pid_motor1.get, pid_motor1.set);
	pid_calc(&pid_motor2, pid_motor2.get, pid_motor2.set);
	
  CAN1_Send_Msg(CAN1,pid_motor1.out,0,0,0);
	CAN2_Send_Msg(CAN2,pid_motor2.out,0,0,0);	
}

