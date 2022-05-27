#include "main.h"
void modeswitch_task ( void )   //在controltask里
{
    get_chassis_mode();
	  get_last_mode();
}

static void get_last_mode ( void ) //底盘的上一个模式
{
    chassis.last_ctrl_mode = chassis.ctrl_mode;
}
void get_chassis_mode ( void )
{
    chassis_mode_handle();
}
void chassis_mode_handle(void)
{
		if ( GetInputMode() == KEY_MOUSE_INPUT )
		{
				chassis.ctrl_mode = CHASSIS_PATROL;
		}
		else if ( GetInputMode() == REMOTE_INPUT)
		{
				chassis.ctrl_mode  =  CHASSIS_REMOTE;
		}
		else if ( GetInputMode() == RELAX)
		{
				chassis.ctrl_mode  =  CHASSIS_RELAX;
		}
		else
		{
				chassis.ctrl_mode  =  CHASSIS_RELAX;
		}
}
