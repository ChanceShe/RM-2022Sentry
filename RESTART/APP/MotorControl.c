#include "main.h"

void motorcontrol_init(void)
{
	PID_struct_init(&pid_motor1 , POSITION_PID , 5000 , 5000 , 50 ,0 ,40);
	PID_struct_init(&pid_motor2 , POSITION_PID , 5000 , 5000 , 50 ,0 ,40);
}

void moter_control(void)//�����������
{	
	pid_motor1.set=800;//CAN1���
  pid_motor2.set=500;//CAN2���
	pid_motor1.get=CM11Encoder.filter_rate;//CAN1���
  pid_motor2.get=CM21Encoder.filter_rate;//CAN2���		
	
	pid_calc(&pid_motor1, pid_motor1.get, pid_motor1.set);
	pid_calc(&pid_motor2, pid_motor2.get, pid_motor2.set);
	
  CAN1_Send_Msg(CAN1,pid_motor1.out,0,0,0);
	CAN2_Send_Msg(CAN2,pid_motor2.out,0,0,0);	
}

