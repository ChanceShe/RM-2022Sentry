#include "main.h"
gimbal_t gim;
float Init_Yaw_Angle  =  0 , Init_Pitch_Angle = 0;
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;

/*****************************		Ѳ��ģʽ����				*************************************/
/******************************   pitch�Ƕȷ�Χ      ************************************/
int PITCH_PERIOD = 260 ;
int16_t pitch_timer = 0;
int8_t pitch_dir = 1;
/******************************    yaw�Ƕȷ�Χ        ***********************************/
int YAW_PERIOD = 1200 ;
int16_t yaw_timer = 0;
int8_t yaw_dir = 1;
/****************************************************************************************/
int8_t dir_yaw = 0;			//yaw����
int8_t dir_pitch = 0;		//pitch����


void gimbal_task(void)
{
	 Hi220_getYawPitchRoll();
	 switch (gim.ctrl_mode)
   {
    case GIMBAL_INIT:							//��̨�س�ʼλ��
      gimbal_init_handle();
      break;
    case GIMBAL_REMOTE_MODE:			//ң��������ģʽ
      gimbal_remote_handle();
      break;
    case GIMBAL_PATROL_MODE:			//Ѳ��ģʽ
      gimbal_patrol_handle();
      break;
		case GIMBAL_NO_ARTI_INPUT:		//������ʼ��
			no_action_handle();
			break;
    default:
      break;
   }
	 
	  VAL_LIMIT ( gim.pid.pit_angle_ref, PITCH_MIN, PITCH_MAX ); //pitch����̨��������
    VAL_LIMIT ( gim.pid.yaw_angle_ref, Init_Yaw_Angle + YAW_MIN, Init_Yaw_Angle + YAW_MAX ); //yaw����̨�Ƕ�����

		pid_calc ( &pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref );
    pid_calc ( &pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref );
    cascade_pid_ctrl();   //����pid����
    pid_calc ( &pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref );
    pid_calc ( &pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref );
		CAN2_Gimbal_Msg (  ( int16_t ) pid_yaw_speed.out, ( int16_t ) pid_pit_speed.out );
}
void cascade_pid_ctrl ( void )
{
    gim.pid.yaw_speed_ref = pid_yaw.out;
    gim.pid.pit_speed_ref = pid_pit.out;
    gim.pid.yaw_speed_fdb = yaw_Gyro;     //-MPU6050_Real_Data.Gyro_Z;  ���ٶ�
    gim.pid.pit_speed_fdb = pitch_Gyro;   //MPU6050_Real_Data.Gyro_X;
}

void gimbal_init_handle( void )		//��̨�س�ʼλ��
{
    int32_t init_rotate_num = 0;

    gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;       //����Ϊ��
    gim.pid.pit_angle_ref = 0;               //-GMPitchEncoder.ecd_angle* (1 -GMPitchRamp.Calc(&GMPitchRamp));
    gim.pid.yaw_angle_fdb = GMYawEncoder.ecd_angle;        //

    init_rotate_num = GMYawEncoder.ecd_angle / 360;
    Init_Yaw_Angle = init_rotate_num * 360;
    gim.pid.yaw_angle_ref = Init_Yaw_Angle;
	

    if ( gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref >= -1.0f && gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref <= 1.0f && gim.pid.pit_angle_fdb >= -1.0f && gim.pid.pit_angle_fdb <= 1.0f ) //��̨�ص���ʼ�ǶȺ����ң����ģʽ
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

        gim.pid.pit_angle_fdb = pitch_Angle;//GMPitchEncoder.ecd_angle;    //����Ϊ��
        gim.pid.yaw_angle_fdb = yaw_Angle;//- GMYawEncoder.ecd_angle;
        GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;		//����������Ϊ��
        GimbalRef.pitch_angle_dynamic_ref = pitch_Angle;	//����������Ϊ��

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
		VAL_LIMIT ( GimbalRef.pitch_angle_dynamic_ref, PITCH_MIN, PITCH_MAX ); //pitch����̨��������
    VAL_LIMIT ( GimbalRef.yaw_angle_dynamic_ref, Init_Yaw_Angle + YAW_MIN, Init_Yaw_Angle + YAW_MAX ); //yaw����̨�Ƕ�����
	  gim.pid.yaw_angle_ref = GimbalRef.yaw_angle_dynamic_ref;
    gim.pid.pit_angle_ref = GimbalRef.pitch_angle_dynamic_ref;
    gim.pid.pit_angle_fdb = pitch_Angle;	//GMPitchEncoder.ecd_angle;    //����Ϊ��
		gim.pid.yaw_angle_fdb = yaw_Angle;		// -GMYawEncoder.ecd_angle;
}
void gimbal_patrol_handle(void)		//Ѳ��ģʽ
{
	  gim.pid.yaw_angle_fdb =  yaw_Angle;    //GMYawEncoder.ecd_angle;
    gim.pid.pit_angle_fdb =  pitch_Angle;  // GMPitchEncoder.ecd_angle;               //ʵ��ֵ
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

void gimbal_param_init ( void )		//��̨�����ʼ��
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
	//б�³�ʼ��
    GMPitchRamp.SetScale ( &GMPitchRamp, PREPARE_TIME_TICK_MS );
    GMYawRamp.SetScale ( &GMYawRamp, PREPARE_TIME_TICK_MS );
    GMPitchRamp.ResetCounter ( &GMPitchRamp );
    GMYawRamp.ResetCounter ( &GMYawRamp );
}
