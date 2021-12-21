#include "main.h"
void modeswitch_task ( void )   //��controltask��
{
    get_gimbal_mode();
    get_chassis_mode();
	  get_gimbal_last_mode();
}
static action_mode_e remote_is_action ( void )
{
    if ( IsRemoteBeingAction() )
    {
        return IS_ACTION;
    }
    else
    {
        return NO_ACTION;
    }
}

void get_gimbal_mode ( void )  // ������������һ�����ģʽ����̨
{
    gim.input.ac_mode = remote_is_action();
    if ( gim.ctrl_mode != GIMBAL_INIT )
    {
        gimbal_mode_handle();    //������̨ģʽ
    }

//  /* gimbal back to center */
    if ( gim.last_ctrl_mode == GIMBAL_RELAX && gim.ctrl_mode != GIMBAL_RELAX )
    {
        gim.ctrl_mode = GIMBAL_INIT;
        //б�³�ʼ��
        GMPitchRamp.SetScale ( &GMPitchRamp, PREPARE_TIME_TICK_MS );
        GMYawRamp.SetScale ( &GMYawRamp, PREPARE_TIME_TICK_MS );
        GMPitchRamp.ResetCounter ( &GMPitchRamp );
        GMYawRamp.ResetCounter ( &GMYawRamp );
        //��̨�����Ƕȳ�ʼ��
			  GimbalRef.pitch_angle_dynamic_ref = 0.0f;
        GimbalRef.yaw_angle_dynamic_ref = 0.0f;
    }
}

static void get_gimbal_last_mode ( void ) //��̨�͵��̵���һ��ģʽ
{
    gim.last_ctrl_mode = gim.ctrl_mode;
    chassis.last_ctrl_mode = chassis.ctrl_mode;
}
void get_chassis_mode ( void )
{
		    if ( gim.ctrl_mode == GIMBAL_INIT )
    {
        chassis.ctrl_mode = CHASSIS_STOP;
    }
    else
    {
        chassis_mode_handle();
    }

}
void gimbal_mode_handle ( void )    //��̨ģʽ�л�
{
    switch ( gim.ctrl_mode )
    {
        case GIMBAL_RELAX:    //(0)

        break;
				
        case GIMBAL_INIT:    //(1)

        break;

        case GIMBAL_NO_ARTI_INPUT:   //(2)
            if ( GetInputMode() == REMOTE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_INIT;
                GMPitchRamp.ResetCounter ( &GMPitchRamp );
                if ( gim.last_ctrl_mode != GIMBAL_REMOTE_MODE )
                {
                    GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;//-GMYawEncoder.ecd_angle;
                }
            }
            else if ( GetInputMode() == KEY_MOUSE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_INIT;
            }
        break;

        case GIMBAL_REMOTE_MODE:     //(3)
            if ( GetInputMode() == REMOTE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_REMOTE_MODE;
            }
            else if ( GetInputMode() == KEY_MOUSE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_PATROL_MODE;
            }
        break;

        case GIMBAL_PATROL_MODE:    //(5)
            if ( GetInputMode() == REMOTE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_INIT;
                GMPitchRamp.ResetCounter ( &GMPitchRamp );
                if ( gim.last_ctrl_mode != GIMBAL_REMOTE_MODE )
                {
                    GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;       //-GMYawEncoder.ecd_angle;
                }
            }
            else if ( GetInputMode() == KEY_MOUSE_INPUT )
            {
                    gim.ctrl_mode = GIMBAL_PATROL_MODE;
            }
        break;
        default:
        break;
    }
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
