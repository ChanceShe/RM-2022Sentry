#include "main.h"
void modeswitch_task ( void )   //��controltask��
{
    get_gimbal_mode();
    get_chassis_mode();
}
void get_gimbal_mode ( void )  // ������������һ�����ģʽ����̨
{
	
}
void get_chassis_mode ( void )
{
//	if ( GetInputMode() == KEY_MOUSE_INPUT )
//    {
//      //vehicle_control_chassis();//����ͨѶ�����ڱ�
//    }
//   else
//     chassis.ctrl_mode  =  CHASSIS_PATROL;
}
