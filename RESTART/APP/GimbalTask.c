#include "main.h"
#include "AHRS_MiddleWare.h"		//����̬����

gimbal_t gim;
float Init_Yaw_Angle  =  0 , Init_Pitch_Angle = 0;
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;

/*****************************		Ѳ��ģʽ����				*************************************/
/******************************   pitch�Ƕȷ�Χ      ************************************/
int PITCH_PERIOD = 200 ;
int16_t pitch_timer = 0;
int8_t pitch_dir = 1;
/******************************    yaw�Ƕȷ�Χ        ***********************************/
int YAW_PERIOD = 1200 ;
int16_t yaw_timer = 0;
int8_t yaw_dir = 1;
/****************************************************************************************/


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
    
		case GIMBAL_AUTO_MODE:				//�Զ�ģʽ
      gimbal_auto_handle();
    break;
		
		case GIMBAL_NO_ARTI_INPUT:		//������ʼ��
			no_action_handle();
		break;
    
		default:
    break;
   }
	 
	 if(gim.ctrl_mode != GIMBAL_RELAX)
	 {
		 VAL_LIMIT ( GimbalRef.pitch_angle_dynamic_ref, PITCH_MIN, PITCH_MAX ); //pitch����̨��������
		 pid_calc ( &pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref );
     pid_calc ( &pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref );
     cascade_pid_ctrl();   			//�0j���pid����
     pid_calc ( &pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref );
     pid_calc ( &pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref );
//		 CAN2_Gimbal_Msg (  ( int16_t ) pid_yaw_speed.out, ( int16_t ) pid_pit_speed.out );
		 CAN2_Gimbal_Msg(0,0);
	 }
	 else
	 {
		 CAN2_Gimbal_Msg(0,0);
	 }
}
void gimbal_param_init ( void )		//��̨�����ʼ��
{
		memset(&gim, 0, sizeof(gimbal_t));
		gim.ctrl_mode      = GIMBAL_NO_ARTI_INPUT;  
		gim.last_ctrl_mode = GIMBAL_RELAX;
		gim.input.ac_mode        = NO_ACTION;
		gim.input.action_angle   = 5.0f;
		Init_Yaw_Angle = yaw_Angle;
		PID_struct_init ( &pid_pit, POSITION_PID , 200, 20,
                      10, 0.05, 0 );
		PID_struct_init ( &pid_pit_speed, POSITION_PID , 29000, 29000,
                      200.0f, 0, 10 );


		PID_struct_init ( &pid_yaw, POSITION_PID , 200, 80,
                      12, 0.05, 0 );
		PID_struct_init ( &pid_yaw_speed, POSITION_PID , 29000, 29000,
                      150.0f, 0, 20 );
	//б�³�ʼ��
    GMPitchRamp.SetScale ( &GMPitchRamp, PREPARE_TIME_TICK_MS );
    GMYawRamp.SetScale ( &GMYawRamp, PREPARE_TIME_TICK_MS );
    GMPitchRamp.ResetCounter ( &GMPitchRamp );
    GMYawRamp.ResetCounter ( &GMYawRamp );
}

void cascade_pid_ctrl ( void )	//����pid����
{
    gim.pid.yaw_speed_ref = pid_yaw.out;
    gim.pid.pit_speed_ref = pid_pit.out;
    gim.pid.yaw_speed_fdb = yaw_Gyro;     //-MPU6050_Real_Data.Gyro_Z;  ���ٶ�
    gim.pid.pit_speed_fdb = pitch_Gyro;   //MPU6050_Real_Data.Gyro_X;
}

void gimbal_init_handle( void )		//��̨�س�ʼλ��
{
    auto_mode = AUTO_PATROL ; //Ѳ�ߣ�û��ʶ��Ŀ��
    int32_t init_rotate_num = 0;
    gim.pid.pit_angle_fdb =  GMPitchEncoder.ecd_angle;    //pitch_Angle;
//    gim.pid.pit_angle_ref = 0 * ( 1 - GMPitchRamp.Calc ( &GMPitchRamp ) ); //GMPitchEncoder.ecd_angle * (1 - GMPitchRamp.Calc(&GMPitchRamp));
    gim.pid.pit_angle_ref = 0;
	//gim.pid.pit_angle_ref = Init_Pitch_Angle  -
    //s												((GMPitchEncoder.ecd_angle - Init_Pitch_Angle)/fabs(GMPitchEncoder.ecd_angle - Init_Pitch_Angle))	* (1 - GMPitchRamp.Calc(&GMPitchRamp));
    gim.pid.yaw_angle_fdb = GMYawEncoder.ecd_angle;
    init_rotate_num = GMYawEncoder.ecd_angle / 360;
    Init_Yaw_Angle = init_rotate_num * 360;
    gim.pid.yaw_angle_ref = Init_Yaw_Angle;                                     //yaw���ʼ�Ƕ�ֵΪ�����0��
    if ( gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref >= 180 )
        gim.pid.yaw_angle_ref += 360;
    else if ( gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref < -180 )
        gim.pid.yaw_angle_ref -= 360;

    if ( gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref >= -1.5f && gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref <= 1.5f && gim.pid.pit_angle_fdb >= -1.5f && gim.pid.pit_angle_fdb <= 1.5f )    //��̨�ص���ʼ�ǶȺ����ң����ģʽ
    {
        /* yaw arrive and switch gimbal state */
        gim.ctrl_mode = GIMBAL_REMOTE_MODE;
        //GimbalRef.yaw_angle_dynamic_ref = ZGyroModuleAngle;
        Init_Yaw_Angle = yaw_Angle;  //-GMYawEncoder.ecd_angle;
        Init_Pitch_Angle = pitch_Angle;
        gim.pid.pit_angle_fdb = pitch_Angle;
        gim.pid.yaw_angle_fdb = yaw_Angle;
        GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;		//����������Ϊ��
        GimbalRef.pitch_angle_dynamic_ref = pitch_Angle;	//����������Ϊ��
    }
}

void no_action_handle ( void )
{
    gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;
    gim.pid.yaw_angle_fdb = 0;
    gim.pid.pit_angle_ref = Init_Pitch_Angle;
    GimbalRef.yaw_angle_dynamic_ref = 0;
}

void gimbal_remote_handle(void)
{
	  gim.pid.yaw_angle_ref = GimbalRef.yaw_angle_dynamic_ref;
    gim.pid.pit_angle_ref = GimbalRef.pitch_angle_dynamic_ref;
    gim.pid.pit_angle_fdb = pitch_Angle;	//GMPitchEncoder.ecd_angle;    //����Ϊ��
		gim.pid.yaw_angle_fdb = yaw_Angle;		// -GMYawEncoder.ecd_angle;
}

auto_mode_e auto_mode = AUTO_PATROL;	//�Զ�ģʽ��ʼΪѲ��״̬
uint8_t auto_lost = 0;
int8_t dir_yaw = 0;			//yaw����
int8_t dir_pitch = 0;		//pitch����
int auto_lost_timer = 0;
uint8_t flag_lost = 0;
float shoot_angle_speed = 0;
float distance_s, distance_x, distance_y;
float x1, x2, x3, x4;
float angle_tan , shoot_radian;

void gimbal_auto_handle(void)
{
	switch ( auto_mode )
	{
		case AUTO_PATROL:
    {
			gimbal_patrol_handle();      //1.  û��ʶ��Ŀ��  ��̨Ѳ��
    }
    break;
    case AUTO_FOLLOW:
    {
      gimbal_follow_handle();      //2.  ʶ��Ŀ�� ����Ŀ��
    }
    break;
    default:
    break;
	}
	auto_shoot_task();										//�Զ��������
}
void gimbal_patrol_handle(void)					//Ѳ��ģʽ
{
	  gim.pid.yaw_angle_fdb =  yaw_Angle;    //�������Ƕ���
    gim.pid.pit_angle_fdb =  pitch_Angle;  
		if ( new_location.id != 0 )                //һ����⵽Ŀ���ͣ��
    {
        auto_mode = AUTO_FOLLOW;
        gim.pid.pit_angle_ref =  pitch_Angle;
        gim.pid.yaw_angle_ref =  yaw_Angle;
			  flag_lost = 0;
        pid_clr ( &pid_yaw );
        pid_clr ( &pid_pit );
        pid_clr ( &pid_yaw_speed );
        pid_clr ( &pid_pit_speed );
    }
    else
    {
        if ( auto_lost )//yaw��ʧ�󱣳�һ��ʱ����˳ʱ���˶�
        {
            auto_lost_timer ++ ;
            if ( auto_lost_timer >= 300 )
            {
                auto_lost = 0;
                auto_lost_timer = 0;
                pitch_timer = ( int16_t ) ( pitch_Angle / ( ( PITCH_MAX - PITCH_MIN ) / PITCH_PERIOD ) );
                yaw_timer = ( int16_t ) ( ( yaw_Angle - Init_Yaw_Angle ) / ( ( YAW_MAX - YAW_MIN ) / YAW_PERIOD ) );
            }
            gim.pid.pit_angle_ref = pitch_Angle;
            gim.pid.yaw_angle_ref = yaw_Angle;
        }
        else
        {
					gim.pid.yaw_angle_ref = yaw_Angle + 5;
//						if ( yaw_dir == 1 )
//						{
//								if ( yaw_timer * ( YAW_MAX - YAW_MIN ) / YAW_PERIOD >= YAW_MAX )
//										yaw_dir = -1;
//								yaw_timer ++ ;
//						}
//						else
//						{
//								if ( yaw_timer * ( YAW_MAX - YAW_MIN ) / YAW_PERIOD <= YAW_MIN )
//										yaw_dir = 1;
//								yaw_timer -- ;
//						}
//						gim.pid.yaw_angle_ref = ( float ) ( Init_Yaw_Angle + (yaw_timer/2) * ( YAW_MAX - YAW_MIN ) / YAW_PERIOD )  ; //*(2*(YAW_MAX-YAW_MIN))/YAW_PERIOD

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
    }

}
Gimbal_Auto_Shoot_t Gimbal_Auto_Shoot;
Speed_Prediction_t Speed_Prediction;

void gimbal_follow_handle(void)		//ʶ��Ŀ�����ģʽ
{
	    if ( new_location.id )
    {
        Gimbal_Auto_Shoot.Recognized_Flag = 1;
        Gimbal_Auto_Shoot.Recognized_Timer = 0;
        Gimbal_Auto_Shoot.Err_Pixels_Yaw	= new_location.x;
        Gimbal_Auto_Shoot.Err_Pixels_Pit	= new_location.y;
        Gimbal_Auto_Shoot.Distance = new_location.dis;
    }
    else
    {
        Gimbal_Auto_Shoot.Recognized_Timer++;
        if ( Gimbal_Auto_Shoot.Recognized_Timer == 100 ) //200ms��ʱ��̫������̨����ԭ�и��������Ҷ���
        {
            Gimbal_Auto_Shoot.Recognized_Flag 	= 0;
            Gimbal_Auto_Shoot.Recognized_Timer 	= 0;
            Gimbal_Auto_Shoot.Err_Pixels_Yaw	= 0;
            Gimbal_Auto_Shoot.Err_Pixels_Pit	= 0;
            Gimbal_Auto_Shoot.Distance = 0;
        }
    }

    if ( Gimbal_Auto_Shoot.Recognized_Flag ) //(new_location.flag)
    {
        Gimbal_Auto_Shoot.Continue_Recognized_Cnt ++;
        VAL_LIMIT ( Gimbal_Auto_Shoot.Continue_Recognized_Cnt, 0, 100 );
        flag_lost = 0;
        gim.pid.yaw_angle_fdb =  yaw_Angle ;
        gim.pid.pit_angle_fdb =  pitch_Angle;

       

        Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample++;
        //-/-------------------- �յ�һ֡ͼ��ʶ������ݣ����д��� ---------------------/-//
        //�����ʱ��֡����ônew_location.x��new_location.yֵ�����ֲ���
				
        if ( new_location.receNewDataFlag )
        {
            new_location.receNewDataFlag = 0;

					 Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( Gimbal_Auto_Shoot.Err_Pixels_Yaw * IMAGE_LENGTH, FOCAL_LENGTH );
           Gimbal_Auto_Shoot.Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( Gimbal_Auto_Shoot.Err_Pixels_Pit * IMAGE_LENGTH, FOCAL_LENGTH );

						Gimbal_Auto_Shoot.Armor_yaw = gim.pid.yaw_angle_fdb + Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw;
						Gimbal_Auto_Shoot.Armor_pit = gim.pid.pit_angle_fdb + Gimbal_Auto_Shoot.Delta_Dect_Angle_Pit;

					  shoot_angle_speed = 27.0;
					
						distance_s =   Gimbal_Auto_Shoot.Distance  / 1000;
						distance_x = ( cos ( Gimbal_Auto_Shoot.Armor_pit * ANGLE_TO_RAD ) * distance_s );
						distance_y = ( sin ( Gimbal_Auto_Shoot.Armor_pit * ANGLE_TO_RAD ) * distance_s );

						x1 = shoot_angle_speed * shoot_angle_speed;
						x2 = distance_x * distance_x;
						x3 = sqrt ( x2 - ( 19.6f * x2 * ( ( 9.8f * x2 ) / ( 2.0f * x1 ) + distance_y ) ) / x1 );
						x4 = 9.8f * x2;
						angle_tan = ( x1 * ( distance_x - x3 ) ) / ( x4 );
						shoot_radian = atan ( angle_tan );
						Gimbal_Auto_Shoot.shoot_pitch_angle = shoot_radian * RAD_TO_ANGLE;
				
           
//            Gimbal_Auto_Shoot.Speed_Prediction.time_delay = 0;//130e-3f;
            //--/----------  ����ǹ�ܾ���Ŀ����ƫ��� ----------------------/--//
            Gimbal_Auto_Shoot.Ballistic_Compensation = ANGLE_BETWEEN_GUN_CAMERA + (RAD_TO_ANGLE * atan2 ( HEIGHT_BETWEEN_GUN_CAMERA , Gimbal_Auto_Shoot.Distance ) );
            Gimbal_Auto_Shoot.Horizontal_Compensation = YAW_ANGLE_BETWEEN_GUN_CAMERA ;
            //Amror_pit װ�װ��pitch��Ƕ�  Amror_yaw װ�װ��yaw��Ƕ�

					  Gimbal_Auto_Shoot.Speed_Prediction.time_delay = distance_x / ( shoot_angle_speed * cos( shoot_radian ) );

            //--/----------------------- ��̨�Ƕȸ��� ----------------------/--//
            gim.pid.yaw_angle_ref = Gimbal_Auto_Shoot.Armor_yaw +  Gimbal_Auto_Shoot.Horizontal_Compensation;
					  gim.pid.pit_angle_ref = Gimbal_Auto_Shoot.shoot_pitch_angle + Gimbal_Auto_Shoot.Ballistic_Compensation;


            //--/--------------�ٶ�Ԥ�⣬����Ŀ��������Լ����ƶ��ٶ�---------/--//
#if ARMY_SPEED_PREDICTION == 1
            if ( Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample % 5 == 0 )
            {
                Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now;                    //�ϴβɼ��ĽǶ�
                Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now = ( Gimbal_Auto_Shoot.Armor_yaw );
                Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now;
                Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now = Gimbal_Auto_Shoot.Armor_pit;
                Gimbal_Auto_Shoot.Speed_Prediction.time1 = Gimbal_Auto_Shoot.Speed_Prediction.time2;
                Gimbal_Auto_Shoot.Speed_Prediction.time2 = time_tick_1ms;
                Gimbal_Auto_Shoot.Speed_Prediction.time_error = Gimbal_Auto_Shoot.Speed_Prediction.time2 - Gimbal_Auto_Shoot.Speed_Prediction.time1;
                Gimbal_Auto_Shoot.Speed_Prediction.yaw_angle_error = Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now - Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Pre;
                Gimbal_Auto_Shoot.Speed_Prediction.pit_angle_error = Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now - Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Pre;
                //���ٶȼ���
                Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw = ( ( Gimbal_Auto_Shoot.Speed_Prediction.yaw_angle_error )	//���ٶȼ���
                        / ( Gimbal_Auto_Shoot.Speed_Prediction.time_error ) ) * 1000;
                Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Pit = ( ( Gimbal_Auto_Shoot.Speed_Prediction.pit_angle_error )
                        / ( Gimbal_Auto_Shoot.Speed_Prediction.time_error ) ) * 1000;



                if ( fabs ( Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw ) < 50 )
                {
                    Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed;
                    Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed = Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw;
                    Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Acceleration = ( ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed
                            - Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed_Pre )
                            / Gimbal_Auto_Shoot.Speed_Prediction.time_error ) * 1000;
                }
                if ( fabs ( Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Pit ) < 50 )
                {
                    Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed;
                    Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed = Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Pit;
                    Gimbal_Auto_Shoot.Speed_Prediction.Pit_Acceleration = ( ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed
                            - Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed_Pre )
                            / Gimbal_Auto_Shoot.Speed_Prediction.time_error ) * 1000;
                }
            }



            Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag = 1;
//            Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample = 0;
#endif
            Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed = AvgFilter ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed );
//            OutData[1] =  ( int ) ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * 100 );
            Gimbal_Auto_Shoot.Speed_Prediction.Continuity_timer = Speed_Continuity_Timmer ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed );
//            OutData[0] = ( int ) ( Gimbal_Auto_Shoot.Armor_yaw * 100 );
//					OutData[3] =  (int)(Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * 100);
        }


        //-/---------------- �������˲���������̨�����Ƕ� ----------------------------/-//
#if ENABLE_KALMAN_FILTER == 1
        kalman_filter_calc ( &kalman_filter_F,
                             gim.pid.yaw_angle_ref,
                             gim.pid.pit_angle_ref,
                             Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed,
                             Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed
                           );
        Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed = kalman_filter_F.filtered_value[2];
        Gimbal_Auto_Shoot.Filtered_Angular_Pit_Speed = kalman_filter_F.filtered_value[3];
//        OutData[1] =  (int)(Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed * 100;
        //Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed = AvgFilter(Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed );
        //OutData[3] =  (int)(Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed * 100);

//        gim.pid.yaw_angle_ref =  kalman_filter_F.filtered_value[0];
//        gim.pid.pit_angle_ref =  kalman_filter_F.filtered_value[1];

#endif

        //-/-------------------- ����ͼ��������̨�����ӳ�������ΪԤ��Ĳ��� ----------------------/-//
#if ARMY_SPEED_PREDICTION == 1
        if ( Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag == 1 )
        {
            Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag = 0;
#if ENABLE_KALMAN_FILTER == 1	           //ʹ�ÿ������˲�֮����ٶȽ��в���������ͼ�����ӳٺ͵����ӳ�
//							Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );
            Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );
            Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation  = ( ( float ) ( ( int ) ( ( Gimbal_Auto_Shoot.Filtered_Angular_Pit_Speed * ( PIT_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay ) + 0.005 ) ) * 100 ) )  / 100  ;

//					    Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
//                                                          					+ Gimbal_Auto_Shoot.Filtered_Yaw_Acceleration
//					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
//					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay ) / 2;
//					    Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
//                                                          					+ Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Acceleration
//					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
//					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay ) / 2;



#else							                       //ʹ��ֱ����õ��ٶȽ��в���	
            Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );
            Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed * ( PIT_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );
#endif

        }

        //-/------------------------ ���벹����������̨�����Ƕ� ----------------------/-//

#if ENABLE_KALMAN_FILTER == 1

        if ( fabs ( Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw ) < 2.0 + fabs ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation ) )
				{
        if(Gimbal_Auto_Shoot.Speed_Prediction.Continuity_timer > 20)
        {
            Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 0;
            gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
            gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
//            sp_flag = 10;

        }
			}
			
//				else
//				{
//					  sp_flag = 0;
//				}
//        OutData[3] = ( int ) ( sp_flag * 100 );
        //ƫ��Ǵ���2�㣬��Ϊ���������
        //1����ʶ�𵽣�ƫ��ϴ����ֱ�Ӳ����ᵼ����̨�����𵴣�
        //2��ʶ��һ��ʱ�䣬���������޲������¸����ͺ�
        //3����̨�ڶ���������yaw�������м���Ծ
        else
        {
            Gimbal_Auto_Shoot.Continue_Large_Err_Cnt++;
            if ( Gimbal_Auto_Shoot.Continue_Large_Err_Cnt >= 150 )  //300ms������300ms��ƫ�������ʱΪ�����ͺ�����벹��
            {
                Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 150;
                gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
                gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
            }
        }


#else
        if ( fabs ( Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw ) < 2.0 + fabs ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation ) )
        {
            Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 0;
            gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
            gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
        }
        //ƫ��Ǵ���2�㣬��Ϊ���������
        //1����ʶ�𵽣�ƫ��ϴ����ֱ�Ӳ����ᵼ����̨�����𵴣�
        //2��ʶ��һ��ʱ�䣬���������޲������¸����ͺ�
        //3����̨�ڶ���������yaw�������м���Ծ
        else
        {
            Gimbal_Auto_Shoot.Continue_Large_Err_Cnt++;
            if ( Gimbal_Auto_Shoot.Continue_Large_Err_Cnt >= 150 )  //300ms������300ms��ƫ�������ʱΪ�����ͺ�����벹��
            {
                Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 150;
                gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
                gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
            }
        }

#endif
        Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation = 0;         //����һ�κ�����
        Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation = 0;
#endif
     
    }
    else
    {

        if ( flag_lost == 0 )
        {
            auto_lost = 1;
            auto_lost_timer = 0;
            auto_mode = AUTO_PATROL;
            flag_lost = 1;
            gim.pid.yaw_angle_ref = yaw_Angle;//GMYawEncoder.ecd_angle ;
            gim.pid.pit_angle_ref = pitch_Angle;//GMPitchEncoder.ecd_angle;

//            pid_clr ( &pid_pit_speed_follow );
//            pid_clr ( &pid_yaw_speed_follow );
//            pid_clr ( &pid_yaw_follow );
//            pid_clr ( &pid_pit_follow );

            pid_clr ( &pid_yaw_speed );
            pid_clr ( &pid_pit_speed );
            pid_clr ( &pid_yaw );
            pid_clr ( &pid_pit );
#if ENABLE_KALMAN_FILTER == 1
            kalman_filter_reset ( &kalman_filter_F, &kalman_filter_I );
#endif
        }
//        gim.pid.yaw_angle_fdb =  yaw_Angle;//-GMYawEncoder.ecd_angle;//����ת���ɽǶ�  ϵ����Ҫ����
//        gim.pid.pit_angle_fdb =  pitch_Angle;//GMPitchEncoder.ecd_angle;//����ת���ɽǶ�  ϵ����Ҫ����

        Gimbal_Auto_Shoot.Continue_Recognized_Cnt = 0;
#if ARMY_SPEED_PREDICTION == 1
        Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample = 0;
        Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed = 0;
        Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed = 0;
        Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation = 0;
        Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation = 0;
        Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag = 0;
        Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 0;
#endif

	}
}
uint32_t losttimer = 0;
uint8_t lostflag = 0;
uint8_t start_preloaded = 0;
uint32_t preloaded_timer = 0;
void auto_shoot_task(void)
{
    if ( !Gimbal_Auto_Shoot.Recognized_Flag )
    {
        losttimer ++ ;
        if ( losttimer >= 500 )
        {
            lostflag = 1;
        }
    }
    else
    {
        losttimer = 0 ;
        lostflag = 0 ;
    }


    if ( gim.pid.yaw_angle_fdb < 2.5f + gim.pid.yaw_angle_ref	\
            && gim.pid.yaw_angle_fdb > -2.5f + gim.pid.yaw_angle_ref )
        dir_yaw = 1;
    else
        dir_yaw = 0;

    if (  gim.pid.pit_angle_fdb < 2.5f + Gimbal_Auto_Shoot.shoot_pitch_angle + Gimbal_Auto_Shoot.Ballistic_Compensation	\
			&& gim.pid.pit_angle_fdb > -2.5f + Gimbal_Auto_Shoot.shoot_pitch_angle + Gimbal_Auto_Shoot.Ballistic_Compensation )
    {
			dir_pitch = 1;
		}
    else
		{
			dir_pitch = 0;
		}

    /*�Զ������������߼�
    	1.�����޷�װ��λ���أ����ȴ�Ħ�����ٴ򿪲��̣����ɱ�������һ���̶��ϵ���ʱ������Ϊ100ms
    	2.�رշ���ʱӦͬʱ�رղ��̺�Ħ����
    	3.װ�׽���һ������ʱ��������״̬AAA
    	4.��Ҫ��ң����״̬�޷��νӱ��ڿ���
    	5.�貦���ض���λ�ſ����������
      6.�ر�֮���ϵ�����ת


    */
    switch ( friction_wheel_state )
    {
        case FRICTION_WHEEL_OFF:
        {
            frictionRamp.ResetCounter ( &frictionRamp );
            if ( RC_CtrlData.rc.s1 != 3  &&  ( new_location.id != 0 ) ) //�ӹرյ�start turning
            {
                SetShootState ( NOSHOOTING );
                friction_rotor = 0;
                frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_TICK_COUNT );
                frictionRamp.ResetCounter ( &frictionRamp );
                friction_wheel_state = FRICTION_WHEEL_START_TURNNING;
            }
        }
        break;
        case FRICTION_WHEEL_START_TURNNING:
        {
            if ( auto_mode == AUTO_PATROL || lostflag == 1 ) //�������ͱ��ر�
            {
                LASER_OFF();
                SetShootState ( NOSHOOTING );
                friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
                frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT );
                frictionRamp.ResetCounter ( &frictionRamp );
                friction_rotor = 2;
            }
            else
            {
                friction_rotor = 1;
                if ( frictionRamp.IsOverflow ( &frictionRamp ) )
                {
                    friction_wheel_state = FRICTION_WHEEL_ON;
                }

            }
        }
        break;
        case FRICTION_WHEEL_ON:
        {

            if (  lostflag  || RC_CtrlData.rc.s1 == 3  )
            {
                friction_rotor = 2;
                friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
                frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT );
                frictionRamp.ResetCounter ( &frictionRamp );
                SetShootState ( NOSHOOTING );
            }
            else if (  ( dir_pitch == 1 ) &&  ( dir_yaw == 1 ) && ( new_location.id != 0 ) ) //&& (judge_rece_mesg.game_state.game_progress == 4) && ( USARTShootFlag == 1)
            {
                SetShootState ( SHOOTING );
                USARTShootFlag = 0;
            }
            else
            {
                SetShootState ( NOSHOOTING );
            }
        }
        break;

        case FRICTION_WHEEL_STOP_TURNNING:
        {
            friction_rotor = 2;
            if ( frictionRamp.IsOverflow ( &frictionRamp ) )
            {
                friction_rotor = 0;
                friction_wheel_state = FRICTION_WHEEL_OFF;
                /*Ԥ������λ���ز���ʹ�������*/

#if  PRELOADED == 1
                SetShootState ( SHOOTING );
                start_preloaded = 1;
                preloaded_timer = 0;
#endif

            }
        }
        break;
    }

}

/******************************************
* input:speed
*	return : Speed_Continuity_Timmer
******************************************/
int Speed_Continuity_Timmer ( float speed )
{
    static int continuity_timer = 0;
    static float last_speed = 0.0;
    if ( ( last_speed > 0 && speed > 0 ) || ( last_speed < 0 && speed < 0 ) )
    {
        continuity_timer ++;
    }
    else
    {
        continuity_timer = 0;
    }
    last_speed = speed;
    return continuity_timer;
}
