#include "main.h"
#include "AHRS_MiddleWare.h"		//大疆姿态解算

gimbal_t gim;
float Init_Yaw_Angle  =  0;
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;

/*****************************		巡逻模式参数				*************************************/
/******************************   pitch角度范围      ************************************/
int PITCH_PERIOD = 100 ;
int16_t pitch_timer = 0;
int8_t pitch_dir = 1;
/******************************    yaw角度范围        ***********************************/
int YAW_PERIOD = 3600 ;
int16_t yaw_timer = 100.0f;
int8_t yaw_dir = 1;
/****************************************************************************************/


void gimbal_task(void)
{
	 CH100_getDATA();
	 switch (gim.ctrl_mode)
   {
    case GIMBAL_INIT:							//云台回初始位置
      gimbal_init_handle();
    break;
    
		case GIMBAL_REMOTE_MODE:			//遥控器控制模式
      gimbal_remote_handle();
    break;
    
		case GIMBAL_AUTO_MODE:				//自动模式
      gimbal_auto_handle();
    break;
		
		case GIMBAL_NO_ARTI_INPUT:		//参数初始化
			no_action_handle();
		break;
    
		default:
    break;
   }
	 
	 if(gim.ctrl_mode == GIMBAL_RELAX)
//	 if(gim.ctrl_mode == GIMBAL_RELAX||gim.ctrl_mode == GIMBAL_AUTO_MODE)
	 {
		 CAN2_Gimbal_Msg(0,0); 
	 }
	 else if(gim.ctrl_mode == GIMBAL_AUTO_MODE)
	 {
		 VAL_LIMIT ( gim.pid.pit_angle_ref, PITCH_MIN, PITCH_MAX );
		 pid_calc ( &pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref );
     pid_calc ( &pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref );
     cascade_pid_ctrl();   			//?j读猵id函数
     pid_calc ( &pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref );
     pid_calc ( &pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref );
		 CAN2_Gimbal_Msg (( int16_t ) pid_yaw_speed.out, ( int16_t ) pid_pit_speed.out );
//		 CAN2_Gimbal_Msg (( int16_t ) pid_yaw_speed.out,0);
//		 CAN2_Gimbal_Msg (0, ( int16_t ) pid_pit_speed.out );
//		 CAN2_Gimbal_Msg(0,0); 
	 }
	 else
	 {
		 VAL_LIMIT ( gim.pid.pit_angle_ref, PITCH_MIN, PITCH_MAX );
		 pid_calc ( &pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref );
     pid_calc ( &pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref );
     cascade_pid_ctrl();   			//�0j读猵id函数
     pid_calc ( &pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref );
     pid_calc ( &pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref );
		 CAN2_Gimbal_Msg (( int16_t ) pid_yaw_speed.out, ( int16_t ) pid_pit_speed.out );
//		 CAN2_Gimbal_Msg (( int16_t ) pid_yaw_speed.out,0);
//		 CAN2_Gimbal_Msg (0, ( int16_t ) pid_pit_speed.out );
//		 CAN2_Gimbal_Msg(0,0); 
	 }
}
void gimbal_param_init ( void )		//云台任务初始化
{
		memset(&gim, 0, sizeof(gimbal_t));
		gim.ctrl_mode      = GIMBAL_NO_ARTI_INPUT;
		gim.last_ctrl_mode = GIMBAL_RELAX;
		gim.input.ac_mode        = NO_ACTION;
		gim.input.action_angle   = 5.0f;
		Init_Yaw_Angle = GMYawEncoder.ecd_angle;
		PID_struct_init ( &pid_pit, POSITION_PID , 150, 20,
                      15, 0.01, 5 );
		PID_struct_init ( &pid_pit_speed, POSITION_PID , 28000, 28000,
                      150.0f, 0.001, 120 );

		PID_struct_init ( &pid_yaw, POSITION_PID , 150, 20,
                      10, 0.02, 30 );
		PID_struct_init ( &pid_yaw_speed, POSITION_PID , 28000, 3000,
                      200.0f, 0, 200 );
	//斜坡初始化
    GMPitchRamp.SetScale ( &GMPitchRamp, PREPARE_TIME_TICK_MS );
    GMYawRamp.SetScale ( &GMYawRamp, PREPARE_TIME_TICK_MS );
    GMPitchRamp.ResetCounter ( &GMPitchRamp );
    GMYawRamp.ResetCounter ( &GMYawRamp );
}

void cascade_pid_ctrl ( void )	//级联pid函数
{
    gim.pid.yaw_speed_ref = pid_yaw.out;
    gim.pid.pit_speed_ref = pid_pit.out;
    gim.pid.yaw_speed_fdb = yaw_Gyro;     //  角速度
    gim.pid.pit_speed_fdb = pitch_Gyro;
}

void gimbal_init_handle( void )		//云台回初始位置
{
    auto_mode = AUTO_PATROL ; //巡逻，没有识别到目标
    int32_t init_rotate_num = 0;
    gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;   //pitch_Angle
//    gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;
//    gim.pid.pit_angle_ref = 0 * ( 1 - GMPitchRamp.Calc ( &GMPitchRamp ) ); //GMPitchEncoder.ecd_angle * (1 - GMPitchRamp.Calc(&GMPitchRamp));
    gim.pid.pit_angle_ref = Init_Pitch_Angle;
	//gim.pid.pit_angle_ref = Init_Pitch_Angle  -
  //s												((GMPitchEncoder.ecd_angle - Init_Pitch_Angle)/fabs(GMPitchEncoder.ecd_angle - Init_Pitch_Angle))	* (1 - GMPitchRamp.Calc(&GMPitchRamp));
    gim.pid.yaw_angle_fdb = GMYawEncoder.ecd_angle;
    init_rotate_num = GMYawEncoder.ecd_angle / 360;
    Init_Yaw_Angle = init_rotate_num * 360;
    gim.pid.yaw_angle_ref = Init_Yaw_Angle;                                     //yaw轴初始角度值为单轴的0度
    if ( gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref >= 180 )
        gim.pid.yaw_angle_ref += 360;
    else if ( gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref < -180 )
        gim.pid.yaw_angle_ref -= 360;
		
    if ( (gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref>=-5.0f) && (gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref<=5.0f) && \
			 (gim.pid.pit_angle_fdb-gim.pid.pit_angle_ref >= -5.0f) && (gim.pid.pit_angle_fdb-gim.pid.pit_angle_ref<=5.0f) )    //云台回到初始角度后进入遥控器模式
		{
        /* yaw arrive and switch gimbal state */
        gim.ctrl_mode = GIMBAL_REMOTE_MODE;
        Init_Yaw_Angle = GMYawEncoder.ecd_angle;  //yaw_Angle;
        gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;
			  gim.pid.yaw_angle_fdb = GMYawEncoder.ecd_angle;
//        gim.pid.yaw_angle_fdb = GMYawEncoder.ecd_angle;
//        GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;		//陀螺仪向右为正
			  GimbalRef.yaw_angle_dynamic_ref = GMYawEncoder.ecd_angle;		//陀螺仪向右为正
//        GimbalRef.pitch_angle_dynamic_ref = pitch_Angle;	//陀螺仪向上为正
        GimbalRef.pitch_angle_dynamic_ref = GMPitchEncoder.ecd_angle;	//陀螺仪向上为正
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
		VAL_LIMIT ( GimbalRef.pitch_angle_dynamic_ref, PITCH_MIN, PITCH_MAX ); //pitch轴云台俯仰限制
	  gim.pid.yaw_angle_ref = GimbalRef.yaw_angle_dynamic_ref;
    gim.pid.pit_angle_ref = GimbalRef.pitch_angle_dynamic_ref;
    gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;	//pitch_Angle;    //向上为正
		gim.pid.yaw_angle_fdb = GMYawEncoder.ecd_angle;		// yaw_Angle;
}

auto_mode_e auto_mode = AUTO_PATROL;	//自动模式初始为巡逻状态
uint8_t auto_lost = 0;
int32_t rotate_num = 0;
int8_t dir_yaw = 0;			//yaw方向
int8_t dir_pitch = 0;		//pitch方向
int auto_lost_timer = 0;
uint8_t flag_lost = 0;
float shoot_angle_speed = 0;
float distance_s, distance_x, distance_y;
float x1, x2, x3, x4;
float angle_tan , shoot_radian;

Gimbal_Auto_Shoot_t Gimbal_Auto_Shoot;
Speed_Prediction_t Speed_Prediction;

void gimbal_auto_handle(void)
{
	switch ( auto_mode )
	{
		case AUTO_PATROL:
    {
			gimbal_patrol_handle();      //1.  没有识别到目标  云台巡逻
    }
    break;
    case AUTO_FOLLOW:
    {
      gimbal_follow_handle();      //2.  识别到目标 跟踪目标
    }
    break;
    default:
    break;
	}
	auto_shoot_task();										//自动射击任务
}

void gimbal_patrol_handle(void)					//巡逻模式
{
	  gim.pid.yaw_angle_fdb =  GMYawEncoder.ecd_angle;    //由陀螺仪读到
    gim.pid.pit_angle_fdb =  GMPitchEncoder.ecd_angle;  
		if ( new_location.recogflag	== 1)                //一旦检测到目标就停下
    {
        auto_mode = AUTO_FOLLOW;
//        gim.pid.pit_angle_ref =  pitch_Angle;
//        gim.pid.yaw_angle_ref =  yaw_Angle;
			  flag_lost = 0;
        pid_clr ( &pid_yaw );
        pid_clr ( &pid_pit );
        pid_clr ( &pid_yaw_speed );
        pid_clr ( &pid_pit_speed );
    }
    else
    {
        if ( auto_lost )//yaw丢失后保持一段时间在顺时针运动
        {
						if(auto_lost_timer == 0)
						{
								gim.pid.pit_lost_feb	= GMPitchEncoder.ecd_angle;
								gim.pid.yaw_lost_feb	= GMYawEncoder.ecd_angle;
						}
            auto_lost_timer ++ ;
            if ( auto_lost_timer >= 6 )
            {
                auto_lost = 0;
                auto_lost_timer = 0;
                pitch_timer = ( int16_t ) ( GMPitchEncoder.ecd_angle / ( ( PITCH_MAX - PITCH_MIN ) / PITCH_PERIOD ) );
                yaw_timer = ( int16_t ) ( ( GMYawEncoder.ecd_angle - Init_Yaw_Angle ) / ( ( YAW_MAX - YAW_MIN ) / YAW_PERIOD ) );
            }
            gim.pid.pit_angle_ref = gim.pid.pit_lost_feb;
            gim.pid.yaw_angle_ref = gim.pid.yaw_lost_feb;
        }
        else
        {
					rotate_num = ( GMYawEncoder.ecd_angle - Init_Yaw_Angle ) / 360;
					gim.pid.yaw_angle_ref = gim.pid.yaw_angle_ref + 0.8;

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


void gimbal_follow_handle(void)		//识别到目标跟随模式
{
	  if ( new_location.recogflag	== 1 )
    {
        Gimbal_Auto_Shoot.Recognized_Flag = 1;
        Gimbal_Auto_Shoot.Recognized_Timer = 0;

				if(fabs(new_location.pitch - Gimbal_Auto_Shoot.target_pit)<=20)
						Gimbal_Auto_Shoot.target_pit = new_location.pitch;
				else
						Gimbal_Auto_Shoot.target_pit = 0;
				
        Gimbal_Auto_Shoot.target_yaw	= new_location.yaw;
    }
    else
    {
        Gimbal_Auto_Shoot.Recognized_Timer++;
        if ( Gimbal_Auto_Shoot.Recognized_Timer == 20 ) //200ms，时间太长，云台保持原有给定导致乱动作
        {
            Gimbal_Auto_Shoot.Recognized_Flag 	= 0;
            Gimbal_Auto_Shoot.Recognized_Timer 	= 0;
            Gimbal_Auto_Shoot.target_yaw	= 0;
            Gimbal_Auto_Shoot.target_pit	= 0;
        }
    }

    if ( Gimbal_Auto_Shoot.Recognized_Flag )
    {
        Gimbal_Auto_Shoot.Continue_Recognized_Cnt ++;
        VAL_LIMIT ( Gimbal_Auto_Shoot.Continue_Recognized_Cnt, 0, 200 );
        flag_lost = 0;
        gim.pid.yaw_angle_fdb =  GMYawEncoder.ecd_angle;			//yaw_Angle ;
        gim.pid.pit_angle_fdb =  GMPitchEncoder.ecd_angle;		//pitch_Angle;

       

        //-/-------------------- 收到一帧图像识别的数据，进行处理 ---------------------/-//
        //如果此时丢帧，那么new_location.x和new_location.y值将保持不变
				
        if ( new_location.recogflag )
        {
//						if(Gimbal_Auto_Shoot.Continue_Recognized_Cnt>=30)
//						{
//							Gimbal_Auto_Shoot.target_pit = AvgFilter(Gimbal_Auto_Shoot.target_pit);
//						}
//						gim.pid.yaw_angle_ref = gim.pid.yaw_angle_fdb + Gimbal_Auto_Shoot.target_yaw ;
//					  gim.pid.pit_angle_ref = gim.pid.pit_angle_fdb - Gimbal_Auto_Shoot.target_pit ;
						gim.pid.yaw_angle_ref = Gimbal_Auto_Shoot.target_yaw ;
					  gim.pid.pit_angle_ref = -( Gimbal_Auto_Shoot.target_pit + 90 + 8) ;

//						gim.pid.yaw_angle_ref = Gimbal_Auto_Shoot.Armor_yaw + Gimbal_Auto_Shoot.Horizontal_Compensation ;
//					  gim.pid.pit_angle_ref =

//						Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( Gimbal_Auto_Shoot.Err_Pixels_Yaw * IMAGE_LENGTH, FOCAL_LENGTH );
//						Gimbal_Auto_Shoot.Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( Gimbal_Auto_Shoot.Err_Pixels_Pit * IMAGE_LENGTH, FOCAL_LENGTH );

//						Gimbal_Auto_Shoot.Armor_yaw = gim.pid.yaw_angle_fdb - Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw;
//						Gimbal_Auto_Shoot.Armor_pit = gim.pid.pit_angle_fdb - Gimbal_Auto_Shoot.Delta_Dect_Angle_Pit; 

//					  shoot_angle_speed = 27.0;
//					
//						distance_s =   Gimbal_Auto_Shoot.Distance  / 1000;
//						distance_x = ( cos ( Gimbal_Auto_Shoot.Armor_pit * ANGLE_TO_RAD ) * distance_s );
//						distance_y = ( sin ( Gimbal_Auto_Shoot.Armor_pit * ANGLE_TO_RAD ) * distance_s );

//						x1 = shoot_angle_speed * shoot_angle_speed;
//						x2 = distance_x * distance_x;
//						x3 = sqrt ( x2 - ( 19.6f * x2 * ( ( 9.8f * x2 ) / ( 2.0f * x1 ) + distance_y ) ) / x1 );
//						x4 = 9.8f * x2;
//						angle_tan = ( x1 * ( distance_x - x3 ) ) / ( x4 );
//						shoot_radian = atan ( angle_tan );
//						Gimbal_Auto_Shoot.shoot_pitch_angle = shoot_radian * RAD_TO_ANGLE;
//				
//           
////            Gimbal_Auto_Shoot.Speed_Prediction.time_delay = 0;//130e-3f;
//            //--/----------  计算枪管距离目标点的偏差角 ----------------------/--//
//            Gimbal_Auto_Shoot.Horizontal_Compensation = YAW_ANGLE_BETWEEN_GUN_CAMERA ;
//						Gimbal_Auto_Shoot.Ballistic_Compensation = ANGLE_BETWEEN_GUN_CAMERA - RAD_TO_ANGLE * atan2 ( HEIGHT_BETWEEN_GUN_CAMERA , Gimbal_Auto_Shoot.Distance );
//            //Amror_pit 装甲板的pitch轴角度  Amror_yaw 装甲板的yaw轴角度

//					  Gimbal_Auto_Shoot.Speed_Prediction.time_delay = distance_x / ( shoot_angle_speed * cos( shoot_radian ) );

//            //--/----------------------- 云台角度给定 ----------------------/--//
//            gim.pid.yaw_angle_ref = Gimbal_Auto_Shoot.Armor_yaw + Gimbal_Auto_Shoot.Horizontal_Compensation ;
//					  gim.pid.pit_angle_ref = Gimbal_Auto_Shoot.Armor_pit + Gimbal_Auto_Shoot.Ballistic_Compensation  ;


//            //--/--------------速度预测，计算目标相对于自己的移动速度---------/--//
//#if ARMY_SPEED_PREDICTION == 1
//            if ( Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample % 1 == 0 )
//            {
//                Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now;                    //上次采集的角度
//                Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now = ( Gimbal_Auto_Shoot.Armor_yaw );																			//本次采集的角度
//                Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now;
//                Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now = Gimbal_Auto_Shoot.Armor_pit;
//                Gimbal_Auto_Shoot.Speed_Prediction.time1 = Gimbal_Auto_Shoot.Speed_Prediction.time2;																		//上次采集的时间
//                Gimbal_Auto_Shoot.Speed_Prediction.time2 = time_tick_1ms;																																//本次采集的时间
//                Gimbal_Auto_Shoot.Speed_Prediction.time_error = Gimbal_Auto_Shoot.Speed_Prediction.time2 - Gimbal_Auto_Shoot.Speed_Prediction.time1;
//                Gimbal_Auto_Shoot.Speed_Prediction.yaw_angle_error = Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now - Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Pre;
//                Gimbal_Auto_Shoot.Speed_Prediction.pit_angle_error = Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now - Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Pre;
//               
//                Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw = ( ( Gimbal_Auto_Shoot.Speed_Prediction.yaw_angle_error )						//角速度计算
//                        / ( Gimbal_Auto_Shoot.Speed_Prediction.time_error ) ) * 1000;
//                Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Pit = ( ( Gimbal_Auto_Shoot.Speed_Prediction.pit_angle_error )
//                        / ( Gimbal_Auto_Shoot.Speed_Prediction.time_error ) ) * 1000;


////                if ( fabs ( Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw ) < 50 )
//                if ( fabs ( Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw ) < 100 )
//                {
//                    Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed;		//上次的角速度
//                    Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed = Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw;						//本次的角速度
//                    Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Acceleration = ( ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed			//角加速度计算(未使用)
//                            - Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed_Pre )
//                            / Gimbal_Auto_Shoot.Speed_Prediction.time_error ) * 1000;
//                }
//                if ( fabs ( Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Pit ) < 50 )
//                {
//                    Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed;
//                    Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed = Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Pit;
//                    Gimbal_Auto_Shoot.Speed_Prediction.Pit_Acceleration = ( ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed
//                            - Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed_Pre )
//                            / Gimbal_Auto_Shoot.Speed_Prediction.time_error ) * 1000;
//                }
//            }


//            Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag = 1;
////            Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample = 0;
//#endif
//            Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed = AvgFilter ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed );
////            OutData[1] =  ( int ) ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * 100 );
//            Gimbal_Auto_Shoot.Speed_Prediction.Continuity_timer = Speed_Continuity_Timmer ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed );
////            OutData[0] = ( int ) ( Gimbal_Auto_Shoot.Armor_yaw * 100 );
////					OutData[3] =  (int)(Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * 100);
        }


//        //-/---------------- 卡尔曼滤波并更新云台给定角度----------------------------/-//
//#if ENABLE_KALMAN_FILTER == 1
//        kalman_filter_calc ( &kalman_filter_F,
//                             gim.pid.yaw_angle_ref,
//                             gim.pid.pit_angle_ref,
//                             Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed,
//                             Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed
//                           );
//        Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed = kalman_filter_F.filtered_value[2];
//        Gimbal_Auto_Shoot.Filtered_Angular_Pit_Speed = kalman_filter_F.filtered_value[3];
////        OutData[1] =  (int)(Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed * 100;
////        Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed = AvgFilter(Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed );
////        OutData[3] =  (int)(Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed * 100);

////        gim.pid.yaw_angle_ref =  kalman_filter_F.filtered_value[0];
////        gim.pid.pit_angle_ref =  kalman_filter_F.filtered_value[1];

//#endif

//        //-/-------------------- 计算图像处理和云台动作延迟量，作为预测的补偿 ----------------------/-//
//#if ARMY_SPEED_PREDICTION == 1
//        if ( Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag == 1 )
//        {
//            Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag = 0;
//#if ENABLE_KALMAN_FILTER == 1	           //使用卡尔曼滤波之后的速度进行补偿，补偿图像处理延迟和弹道延迟
////							Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );
//        Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );
//        Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation  = ( ( float ) ( ( int ) ( ( Gimbal_Auto_Shoot.Filtered_Angular_Pit_Speed * ( PIT_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay ) + 0.005 ) ) * 100 ) )  / 100  ;

////					    Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
////                                                          					+ Gimbal_Auto_Shoot.Filtered_Yaw_Acceleration
////					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
////					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay ) / 2;
////					    Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
////                                                          					+ Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Acceleration
////					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
////					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay ) / 2;



//#else							                       //使用直接求得的速度进行补偿	
//            Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );
//            Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed * ( PIT_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );
//#endif

//        }
//#if ENABLE_KALMAN_FILTER == 1
//				
//        if ( fabs ( Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw ) < 3.0 + fabs ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation ) )
//				{
//					if(Gimbal_Auto_Shoot.Speed_Prediction.Continuity_timer > 20)
//					{
//							Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 0;
//							gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
//							gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
//					}
//				}
//			
//        //偏差角大于2°，分为另种情况。
//        //1、刚识别到，偏差较大，如果直接补偿会导致云台抖动震荡；
//        //2、识别到一定时间，但是由于无补偿导致跟踪滞后
//        //3、云台在抖动，导致yaw反馈在中间跳跃
//        else
//        {
//            Gimbal_Auto_Shoot.Continue_Large_Err_Cnt++;
//            if ( Gimbal_Auto_Shoot.Continue_Large_Err_Cnt >= 40 )  //300ms，持续300ms的偏差，表明此时为跟踪滞后，需加入补偿
//            {
//                Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 40;
//                gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
//                gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
//            }
//        }


//#else
//				
//				  //-/------------------------ 加入补偿，更新云台给定角度 ----------------------/-//

//          //根据偏差角计算偏差距离，由于距离信息存在误差，不使用计算的temp = fabs(Gimbal_Auto_Shoot.Distance * 2.0f * arm_sin_f32(0.5f * fabs(Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw) * 0.01745329252f ));
//          //偏差角度小于2度，说明此时基本瞄准装甲片，两个时间参数应该需要调整。
//          //该处应该加上pitch轴的预测判定
//        if ( fabs ( Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw ) < 3.0 + fabs ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation ) )
//        {
//            Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 0;
//            gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
//            gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
//        }
//        //偏差角大于2°，分为另种情况。
//        //1、刚识别到，偏差较大，如果直接补偿会导致云台抖动震荡；
//        //2、识别到一定时间，但是由于无补偿导致跟踪滞后
//        //3、云台在抖动，导致yaw反馈在中间跳跃
//        else
//        {
//            Gimbal_Auto_Shoot.Continue_Large_Err_Cnt++;
//            if ( Gimbal_Auto_Shoot.Continue_Large_Err_Cnt >=100 )  //300ms，持续300ms的偏差，表明此时为跟踪滞后，需加入补偿
//            {
//                Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 100;
//                gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
//                gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
//            }
//        }

//#endif
//        Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation = 0;         //补偿一次后清零
//        Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation = 0;
//#endif
     
    }
    else
    {

        if ( flag_lost == 0 )
        {
            auto_lost = 1;
            auto_lost_timer = 0;
            auto_mode = AUTO_PATROL;
            flag_lost = 1;
            gim.pid.yaw_angle_ref = GMYawEncoder.ecd_angle;//yaw_Angle;
            gim.pid.pit_angle_ref = GMPitchEncoder.ecd_angle;//pitch_Angle;

//            pid_clr ( &pid_pit_speed_follow );
//            pid_clr ( &pid_yaw_speed_follow );
//            pid_clr ( &pid_yaw_follow );
//            pid_clr ( &pid_pit_follow );

            pid_clr ( &pid_yaw_speed );
            pid_clr ( &pid_pit_speed );
            pid_clr ( &pid_yaw );
            pid_clr ( &pid_pit );
//#if ENABLE_KALMAN_FILTER == 1
//            kalman_filter_reset ( &kalman_filter_F, &kalman_filter_I );
//#endif
        }
//        gim.pid.yaw_angle_fdb =  yaw_Angle;//-GMYawEncoder.ecd_angle;//最终转换成角度  系数需要调整
//        gim.pid.pit_angle_fdb =  pitch_Angle;//GMPitchEncoder.ecd_angle;//最终转换成角度  系数需要调整

        Gimbal_Auto_Shoot.Continue_Recognized_Cnt = 0;
//#if ARMY_SPEED_PREDICTION == 1
//        Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample = 0;
//        Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed = 0;
//        Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed = 0;
//        Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation = 0;
//        Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation = 0;
//        Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag = 0;
//        Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 0;
//#endif

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
        if ( losttimer >= 50 )
        {
            lostflag = 1;
        }
    }
    else
    {
        losttimer = 0 ;
        lostflag = 0 ;
    }


    if ( gim.pid.yaw_angle_fdb < 5.0f + gim.pid.yaw_angle_ref	\
            && gim.pid.yaw_angle_fdb > -5.0f + gim.pid.yaw_angle_ref )
        dir_yaw = 1;
    else
        dir_yaw = 0;

    if (  gim.pid.pit_angle_fdb < 7.0f + gim.pid.pit_angle_ref	\
            && gim.pid.pit_angle_fdb > -7.0f + gim.pid.pit_angle_ref  )
    {
			dir_pitch = 1;
		}
    else
		{
			dir_pitch = 0;
		}

    /*自动打击发射控制逻辑
    	1.由于无法装限位开关，需先打开摩擦轮再打开拨盘，不可避免会造成一定程度上的延时，初步为100ms
    	2.关闭发射时应同时关闭拨盘和摩擦轮
    	3.装甲进入一定区域时触发发射状态AAA
    	4.需要和遥控器状态无缝衔接便于控制
    	5.需拨至特定档位才开启射击功能
      6.关闭之后上弹至堵转


    */
    switch ( friction_wheel_state )
    {
        case FRICTION_WHEEL_OFF:
        {
            frictionRamp.ResetCounter ( &frictionRamp );
            if ( refromData.s1 != 3  &&  ( new_location.recogflag != 0 ) ) //从关闭到start turning
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
            if ( auto_mode == AUTO_PATROL || lostflag == 1 ) //刚启动就被关闭
            {
//
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

            if (  lostflag  || refromData.s1 == 3  )
            {
                friction_rotor = 2;
                friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
                frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT );
                frictionRamp.ResetCounter ( &frictionRamp );
                SetShootState ( NOSHOOTING );
            }
            else if (  ( dir_pitch == 1 ) &&  ( dir_yaw == 1 ) && ( new_location.recogflag != 0 ) )
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
                /*预案，限位开关不好使就用这个*/

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

