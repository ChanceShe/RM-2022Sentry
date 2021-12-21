#include "main.h"
gimbal_t gim;
float Init_Yaw_Angle  =  0 , Init_Pitch_Angle = 0;
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;

/*****************************		巡逻模式参数				*************************************/
/******************************   pitch角度范围      ************************************/
int PITCH_PERIOD = 260 ;
int16_t pitch_timer = 0;
int8_t pitch_dir = 1;
/******************************    yaw角度范围        ***********************************/
int YAW_PERIOD = 1200 ;
int16_t yaw_timer = 0;
int8_t yaw_dir = 1;
/****************************************************************************************/
int8_t dir_yaw = 0;			//yaw方向
int8_t dir_pitch = 0;		//pitch方向


void gimbal_task(void)
{
	 Hi220_getYawPitchRoll();
	 switch (gim.ctrl_mode)
   {
    case GIMBAL_INIT:							//云台回初始位置
      gimbal_init_handle();
      break;
    case GIMBAL_REMOTE_MODE:			//遥控器控制模式
      gimbal_remote_handle();
      break;
    case GIMBAL_PATROL_MODE:			//巡逻模式
      gimbal_patrol_handle();
      break;
		case GIMBAL_NO_ARTI_INPUT:		//参数初始化
			no_action_handle();
			break;
    default:
      break;
   }
	 
	  VAL_LIMIT ( gim.pid.pit_angle_ref, PITCH_MIN, PITCH_MAX ); //pitch轴云台俯仰限制
    VAL_LIMIT ( gim.pid.yaw_angle_ref, Init_Yaw_Angle + YAW_MIN, Init_Yaw_Angle + YAW_MAX ); //yaw轴云台角度限制

		pid_calc ( &pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref );
    pid_calc ( &pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref );
    cascade_pid_ctrl();   //级联pid函数
    pid_calc ( &pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref );
    pid_calc ( &pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref );
		CAN2_Gimbal_Msg (  ( int16_t ) pid_yaw_speed.out, ( int16_t ) pid_pit_speed.out );
}
void cascade_pid_ctrl ( void )
{
    gim.pid.yaw_speed_ref = pid_yaw.out;
    gim.pid.pit_speed_ref = pid_pit.out;
    gim.pid.yaw_speed_fdb = yaw_Gyro;     //-MPU6050_Real_Data.Gyro_Z;  角速度
    gim.pid.pit_speed_fdb = pitch_Gyro;   //MPU6050_Real_Data.Gyro_X;
}

void gimbal_init_handle( void )		//云台回初始位置
{
    int32_t init_rotate_num = 0;

    gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;       //向上为负
    gim.pid.pit_angle_ref = 0;               //-GMPitchEncoder.ecd_angle* (1 -GMPitchRamp.Calc(&GMPitchRamp));
    gim.pid.yaw_angle_fdb = GMYawEncoder.ecd_angle;        //

    init_rotate_num = GMYawEncoder.ecd_angle / 360;
    Init_Yaw_Angle = init_rotate_num * 360;
    gim.pid.yaw_angle_ref = Init_Yaw_Angle;
	

    if ( gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref >= -1.0f && gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref <= 1.0f && gim.pid.pit_angle_fdb >= -1.0f && gim.pid.pit_angle_fdb <= 1.0f ) //云台回到初始角度后进入遥控器模式
    {
        /* yaw arrive and switch gimbal state */
        if ( GetInputMode() == REMOTE_INPUT )
        {
            gim.ctrl_mode = GIMBAL_REMOTE_MODE
					;
        }
        if ( GetInputMode() == KEY_MOUSE_INPUT )
        {
            gim.ctrl_mode = GIMBAL_PATROL_MODE;
        }
        Init_Yaw_Angle = yaw_Angle;  //-GMYawEncoder.ecd_angle;
        Init_Pitch_Angle = pitch_Angle;

        gim.pid.pit_angle_fdb = pitch_Angle;//GMPitchEncoder.ecd_angle;    //向上为正
        gim.pid.yaw_angle_fdb = yaw_Angle;//- GMYawEncoder.ecd_angle;
        GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;		//陀螺仪向右为正
        GimbalRef.pitch_angle_dynamic_ref = pitch_Angle;	//陀螺仪向上为正

    }
}

void no_action_handle(void)
{
	  gim.pid.pit_angle_fdb = 0;//GMPitchEncoder.ecd_angle;
    gim.pid.yaw_angle_fdb = 0;//GMYawEncoder.ecd_angle;// GMYawEncoder.ecd_angle;
    GimbalRef.pitch_angle_dynamic_ref = 0;
    GimbalRef.yaw_angle_dynamic_ref = 0;
}

void gimbal_remote_handle(void)
{
		VAL_LIMIT ( GimbalRef.pitch_angle_dynamic_ref, PITCH_MIN, PITCH_MAX ); //pitch轴云台俯仰限制
    VAL_LIMIT ( GimbalRef.yaw_angle_dynamic_ref, Init_Yaw_Angle + YAW_MIN, Init_Yaw_Angle + YAW_MAX ); //yaw轴云台角度限制
	  gim.pid.yaw_angle_ref = GimbalRef.yaw_angle_dynamic_ref;
    gim.pid.pit_angle_ref = GimbalRef.pitch_angle_dynamic_ref;
    gim.pid.pit_angle_fdb = pitch_Angle;	//GMPitchEncoder.ecd_angle;    //向上为正
		gim.pid.yaw_angle_fdb = yaw_Angle;		// -GMYawEncoder.ecd_angle;
}
void gimbal_patrol_handle(void)		//巡逻模式
{
	  gim.pid.yaw_angle_fdb =  yaw_Angle;    //GMYawEncoder.ecd_angle;
    gim.pid.pit_angle_fdb =  pitch_Angle;  // GMPitchEncoder.ecd_angle;               //实际值
		if ( yaw_dir == 1 )
		{
				if ( yaw_timer * ( YAW_MAX - YAW_MIN ) / YAW_PERIOD >= YAW_MAX )
						yaw_dir = -1;
				yaw_timer ++ ;
		}
		else
		{
				if ( yaw_timer * ( YAW_MAX - YAW_MIN ) / YAW_PERIOD <= YAW_MIN )
						yaw_dir = 1;
				yaw_timer -- ;
		}
		gim.pid.yaw_angle_ref = ( float ) ( Init_Yaw_Angle + yaw_timer * ( YAW_MAX - YAW_MIN ) / YAW_PERIOD )  ; //*(2*(YAW_MAX-YAW_MIN))/YAW_PERIOD

		if ( pitch_dir == 1 )
		{
				if ( pitch_timer * ( PITCH_MAX - PITCH_MIN ) / PITCH_PERIOD >= PITCH_MAX )
						pitch_dir = -1;
				pitch_timer ++ ;
		}
		else
		{
				if ( pitch_timer * ( PITCH_MAX - PITCH_MIN ) / PITCH_PERIOD <= PITCH_MIN )
						pitch_dir = 1;
				pitch_timer -- ;
		}
		gim.pid.pit_angle_ref = ( float ) ( pitch_timer * ( PITCH_MAX - PITCH_MIN ) / PITCH_PERIOD );

}

void gimbal_param_init ( void )		//云台任务初始化
{
		memset(&gim, 0, sizeof(gimbal_t));
		gim.ctrl_mode      = GIMBAL_NO_ARTI_INPUT;  
		gim.last_ctrl_mode = GIMBAL_RELAX;
		gim.input.ac_mode        = NO_ACTION;
		gim.input.action_angle   = 5.0f;
		Init_Pitch_Angle = 0;
		Init_Yaw_Angle = 80;
		PID_struct_init ( &pid_pit, POSITION_PID , 200, 20,
                      10, 0.05, 0 );
		PID_struct_init ( &pid_pit_speed, POSITION_PID , 29000, 29000,
                      300, 0, 10 );


		PID_struct_init ( &pid_yaw, POSITION_PID , 200, 80,
                      12, 0.05, 0 );
		PID_struct_init ( &pid_yaw_speed, POSITION_PID , 29000, 29000,
                      200.0f, 0, 0 );
	//斜坡初始化
    GMPitchRamp.SetScale ( &GMPitchRamp, PREPARE_TIME_TICK_MS );
    GMYawRamp.SetScale ( &GMYawRamp, PREPARE_TIME_TICK_MS );
    GMPitchRamp.ResetCounter ( &GMPitchRamp );
    GMYawRamp.ResetCounter ( &GMYawRamp );
}
