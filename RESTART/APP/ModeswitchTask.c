#include "main.h"
void modeswitch_task ( void )   //��controltask��
{
    get_chassis_mode();
	  get_last_mode();
}

static void get_last_mode ( void ) //��̨�͵��̵���һ��ģʽ
{
    chassis.last_ctrl_mode = chassis.ctrl_mode;
}
void get_chassis_mode ( void )
{
//		    if ( gim.ctrl_mode == GIMBAL_INIT )
//    {
//        chassis.ctrl_mode = CHASSIS_STOP;
//    }
//    else
//    {
        chassis_mode_handle();
//    }

}
void chassis_mode_handle(void)
{
		if ( GetInputMode() == KEY_MOUSE_INPUT )
		{
				chassis.ctrl_mode = CHASSIS_PATROL;
		}
		else
				chassis.ctrl_mode  =  CHASSIS_REMOTE;

}
