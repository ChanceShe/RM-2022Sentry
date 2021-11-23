#include "main.h"
void modeswitch_task ( void )   //在controltask里
{
    get_gimbal_mode();
    get_chassis_mode();
}
void get_gimbal_mode ( void )  // 从上往下数第一个获得模式，云台
{
	
}
void get_chassis_mode ( void )
{
//	if ( GetInputMode() == KEY_MOUSE_INPUT )
//    {
//      //vehicle_control_chassis();//车间通讯控制哨兵
//    }
//   else
//     chassis.ctrl_mode  =  CHASSIS_PATROL;
}
