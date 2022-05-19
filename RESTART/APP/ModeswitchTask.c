#include "main.h"
void modeswitch_task ( void )   //在controltask里
{
    get_gimbal_mode();
	  get_last_mode();
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

void get_gimbal_mode ( void )  // 从上往下数第一个获得模式，云台
{
    gim.input.ac_mode = remote_is_action();
    if ( gim.ctrl_mode != GIMBAL_INIT )
    {
        gimbal_mode_handle();    //进入云台模式
    }
		else
		{
			if ( GetInputMode() == STOP )
			{
				gim.ctrl_mode = GIMBAL_RELAX;
			}
		}
//  /* gimbal back to center */
    if ( gim.last_ctrl_mode == GIMBAL_RELAX && gim.ctrl_mode != GIMBAL_RELAX )
    {
        gim.ctrl_mode = GIMBAL_INIT;
        //斜坡初始化
        GMPitchRamp.SetScale ( &GMPitchRamp, PREPARE_TIME_TICK_MS );
        GMYawRamp.SetScale ( &GMYawRamp, PREPARE_TIME_TICK_MS );
        GMPitchRamp.ResetCounter ( &GMPitchRamp );
        GMYawRamp.ResetCounter ( &GMYawRamp );
        //云台给定角度初始化
        GimbalRef.pitch_angle_dynamic_ref = 0.0f;
        GimbalRef.yaw_angle_dynamic_ref = 0.0f;
    }
}

static void get_last_mode ( void ) //云台和底盘的上一个模式
{
    gim.last_ctrl_mode = gim.ctrl_mode;
}

void gimbal_mode_handle ( void )    //云台模式切换
{
    switch ( gim.ctrl_mode )
    {
        case GIMBAL_RELAX:    //(0)
						if ( GetInputMode() == REMOTE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_INIT;
                GMPitchRamp.ResetCounter ( &GMPitchRamp );
                if ( gim.last_ctrl_mode != GIMBAL_REMOTE_MODE )
                {
                    gim.pid.pit_angle_ref = GMPitchEncoder.ecd_angle;	//pitch_Angle;
									  gim.pid.yaw_angle_ref = GMYawEncoder.ecd_angle;	//yaw_Angle;
                }
            }
            else if ( GetInputMode() == KEY_MOUSE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_AUTO_MODE;
            }
						else
						{
								gim.ctrl_mode = GIMBAL_RELAX;
						}
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
                    Init_Yaw_Angle = GMYawEncoder.ecd_angle;//-GMYawEncoder.ecd_angle;
                }
            }
            else if ( GetInputMode() == KEY_MOUSE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_AUTO_MODE;
            }
						else
						{
								gim.ctrl_mode = GIMBAL_RELAX;
						}
        break;

        case GIMBAL_REMOTE_MODE:     //(3)
            if ( GetInputMode() == REMOTE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_REMOTE_MODE;
								if ( gim.last_ctrl_mode != GIMBAL_REMOTE_MODE )
									{
											Init_Yaw_Angle = GMYawEncoder.ecd_angle;//-GMYawEncoder.ecd_angle;
									}
						}
            else if ( GetInputMode() == KEY_MOUSE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_AUTO_MODE;
            }
						else
						{
								gim.ctrl_mode = GIMBAL_RELAX;
						}
        break;

        case GIMBAL_AUTO_MODE:    //(5)
            if ( GetInputMode() == REMOTE_INPUT )
            {
                gim.ctrl_mode = GIMBAL_INIT;
                GMPitchRamp.ResetCounter ( &GMPitchRamp );
                if ( gim.last_ctrl_mode != GIMBAL_REMOTE_MODE )
                {
                    GimbalRef.yaw_angle_dynamic_ref =-GMYawEncoder.ecd_angle;//yaw_Angle; 
                }
            }
            else if ( GetInputMode() == KEY_MOUSE_INPUT )
            {
                    gim.ctrl_mode = GIMBAL_AUTO_MODE;
            }
						else
						{
								gim.ctrl_mode = GIMBAL_RELAX;
						}
        break;
        default:
        break;
    }
}
