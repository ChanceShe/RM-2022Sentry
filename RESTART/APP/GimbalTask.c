#include "main.h"
#include "AHRS_MiddleWare.h"		//´ó½®×ËÌ¬½âËã

gimbal_t gim;
float Init_Yaw_Angle  =  0;
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;

/*****************************		Ñ²ÂßÄ£Ê½²ÎÊý				*************************************/
/******************************   pitch½Ç¶È·¶Î§      ************************************/
int PITCH_PERIOD = 100 ;
int16_t pitch_timer = 0;
int8_t pitch_dir = 1;
/******************************    yaw½Ç¶È·¶Î§        ***********************************/
int YAW_PERIOD = 3600 ;
int16_t yaw_timer = 100.0f;
int8_t yaw_dir = 1;
/****************************************************************************************/


void gimbal_task(void)
{
	 CH100_getDATA();
	 switch (gim.ctrl_mode)
   {
    case GIMBAL_INIT:							//ÔÆÌ¨»Ø³õÊ¼Î»ÖÃ
      gimbal_init_handle();
    break;
    
		case GIMBAL_REMOTE_MODE:			//Ò£¿ØÆ÷¿ØÖÆÄ£Ê½
      gimbal_remote_handle();
    break;
    
		case GIMBAL_AUTO_MODE:				//×Ô¶¯Ä£Ê½
      gimbal_auto_handle();
    break;
		
		case GIMBAL_NO_ARTI_INPUT:		//²ÎÊý³õÊ¼»¯
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
     cascade_pid_ctrl();   			//?j¶Áªpidº¯Êý
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
     cascade_pid_ctrl();   			//¼0j¶Áªpidº¯Êý
     pid_calc ( &pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref );
     pid_calc ( &pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref );
		 CAN2_Gimbal_Msg (( int16_t ) pid_yaw_speed.out, ( int16_t ) pid_pit_speed.out );
//		 CAN2_Gimbal_Msg (( int16_t ) pid_yaw_speed.out,0);
//		 CAN2_Gimbal_Msg (0, ( int16_t ) pid_pit_speed.out );
//		 CAN2_Gimbal_Msg(0,0); 
	 }
}
void gimbal_param_init ( void )		//ÔÆÌ¨ÈÎÎñ³õÊ¼»¯
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
	//Ð±ÆÂ³õÊ¼»¯
    GMPitchRamp.SetScale ( &GMPitchRamp, PREPARE_TIME_TICK_MS );
    GMYawRamp.SetScale ( &GMYawRamp, PREPARE_TIME_TICK_MS );
    GMPitchRamp.ResetCounter ( &GMPitchRamp );
    GMYawRamp.ResetCounter ( &GMYawRamp );
}

void cascade_pid_ctrl ( void )	//¼¶Áªpidº¯Êý
{
    gim.pid.yaw_speed_ref = pid_yaw.out;
    gim.pid.pit_speed_ref = pid_pit.out;
    gim.pid.yaw_speed_fdb = yaw_Gyro;     //  ½ÇËÙ¶È
    gim.pid.pit_speed_fdb = pitch_Gyro;
}

void gimbal_init_handle( void )		//ÔÆÌ¨»Ø³õÊ¼Î»ÖÃ
{
    auto_mode = AUTO_PATROL ; //Ñ²Âß£¬Ã»ÓÐÊ¶±ðµ½Ä¿±ê
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
    gim.pid.yaw_angle_ref = Init_Yaw_Angle;                                     //yawÖá³õÊ¼½Ç¶ÈÖµÎªµ¥ÖáµÄ0¶È
    if ( gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref >= 180 )
        gim.pid.yaw_angle_ref += 360;
    else if ( gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref < -180 )
        gim.pid.yaw_angle_ref -= 360;
		
    if ( (gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref>=-5.0f) && (gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref<=5.0f) && \
			 (gim.pid.pit_angle_fdb-gim.pid.pit_angle_ref >= -5.0f) && (gim.pid.pit_angle_fdb-gim.pid.pit_angle_ref<=5.0f) )    //ÔÆÌ¨»Øµ½³õÊ¼½Ç¶Èºó½øÈëÒ£¿ØÆ÷Ä£Ê½
		{
        /* yaw arrive and switch gimbal state */
        gim.ctrl_mode = GIMBAL_REMOTE_MODE;
        Init_Yaw_Angle = GMYawEncoder.ecd_angle;  //yaw_Angle;
        gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;
			  gim.pid.yaw_angle_fdb = GMYawEncoder.ecd_angle;
//        gim.pid.yaw_angle_fdb = GMYawEncoder.ecd_angle;
//        GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;		//ÍÓÂÝÒÇÏòÓÒÎªÕý
			  GimbalRef.yaw_angle_dynamic_ref = GMYawEncoder.ecd_angle;		//ÍÓÂÝÒÇÏòÓÒÎªÕý
//        GimbalRef.pitch_angle_dynamic_ref = pitch_Angle;	//ÍÓÂÝÒÇÏòÉÏÎªÕý
        GimbalRef.pitch_angle_dynamic_ref = GMPitchEncoder.ecd_angle;	//ÍÓÂÝÒÇÏòÉÏÎªÕý
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
		VAL_LIMIT ( GimbalRef.pitch_angle_dynamic_ref, PITCH_MIN, PITCH_MAX ); //pitchÖáÔÆÌ¨¸©ÑöÏÞÖÆ
	  gim.pid.yaw_angle_ref = GimbalRef.yaw_angle_dynamic_ref;
    gim.pid.pit_angle_ref = GimbalRef.pitch_angle_dynamic_ref;
    gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;	//pitch_Angle;    //ÏòÉÏÎªÕý
		gim.pid.yaw_angle_fdb = GMYawEncoder.ecd_angle;		// yaw_Angle;
}

auto_mode_e auto_mode = AUTO_PATROL;	//×Ô¶¯Ä£Ê½³õÊ¼ÎªÑ²Âß×´Ì¬
uint8_t auto_lost = 0;
int32_t rotate_num = 0;
int8_t dir_yaw = 0;			//yaw·½Ïò
int8_t dir_pitch = 0;		//pitch·½Ïò
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
			gimbal_patrol_handle();      //1.  Ã»ÓÐÊ¶±ðµ½Ä¿±ê  ÔÆÌ¨Ñ²Âß
    }
    break;
    case AUTO_FOLLOW:
    {
      gimbal_follow_handle();      //2.  Ê¶±ðµ½Ä¿±ê ¸ú×ÙÄ¿±ê
    }
    break;
    default:
    break;
	}
	auto_shoot_task();										//×Ô¶¯Éä»÷ÈÎÎñ
}

void gimbal_patrol_handle(void)					//Ñ²ÂßÄ£Ê½
{
	  gim.pid.yaw_angle_fdb =  GMYawEncoder.ecd_angle;    //ÓÉÍÓÂÝÒÇ¶Áµ½
    gim.pid.pit_angle_fdb =  GMPitchEncoder.ecd_angle;  
		if ( new_location.recogflag	== 1)                //Ò»µ©¼ì²âµ½Ä¿±ê¾ÍÍ£ÏÂ
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
        if ( auto_lost )//yaw¶ªÊ§ºó±£³ÖÒ»¶ÎÊ±¼äÔÚË³Ê±ÕëÔË¶¯
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


void gimbal_follow_handle(void)		//Ê¶±ðµ½Ä¿±ê¸úËæÄ£Ê½
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
        if ( Gimbal_Auto_Shoot.Recognized_Timer == 20 ) //200ms£¬Ê±¼äÌ«³¤£¬ÔÆÌ¨±£³ÖÔ­ÓÐ¸ø¶¨µ¼ÖÂÂÒ¶¯×÷
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

       

        //-/-------------------- ÊÕµ½Ò»Ö¡Í¼ÏñÊ¶±ðµÄÊý¾Ý£¬½øÐÐ´¦Àí ---------------------/-//
        //Èç¹û´ËÊ±¶ªÖ¡£¬ÄÇÃ´new_location.xºÍnew_location.yÖµ½«±£³Ö²»±ä
				
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
//            //--/----------  ¼ÆËãÇ¹¹Ü¾àÀëÄ¿±êµãµÄÆ«²î½Ç ----------------------/--//
//            Gimbal_Auto_Shoot.Horizontal_Compensation = YAW_ANGLE_BETWEEN_GUN_CAMERA ;
//						Gimbal_Auto_Shoot.Ballistic_Compensation = ANGLE_BETWEEN_GUN_CAMERA - RAD_TO_ANGLE * atan2 ( HEIGHT_BETWEEN_GUN_CAMERA , Gimbal_Auto_Shoot.Distance );
//            //Amror_pit ×°¼×°åµÄpitchÖá½Ç¶È  Amror_yaw ×°¼×°åµÄyawÖá½Ç¶È

//					  Gimbal_Auto_Shoot.Speed_Prediction.time_delay = distance_x / ( shoot_angle_speed * cos( shoot_radian ) );

//            //--/----------------------- ÔÆÌ¨½Ç¶È¸ø¶¨ ----------------------/--//
//            gim.pid.yaw_angle_ref = Gimbal_Auto_Shoot.Armor_yaw + Gimbal_Auto_Shoot.Horizontal_Compensation ;
//					  gim.pid.pit_angle_ref = Gimbal_Auto_Shoot.Armor_pit + Gimbal_Auto_Shoot.Ballistic_Compensation  ;


//            //--/--------------ËÙ¶ÈÔ¤²â£¬¼ÆËãÄ¿±êÏà¶ÔÓÚ×Ô¼ºµÄÒÆ¶¯ËÙ¶È---------/--//
//#if ARMY_SPEED_PREDICTION == 1
//            if ( Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample % 1 == 0 )
//            {
//                Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now;                    //ÉÏ´Î²É¼¯µÄ½Ç¶È
//                Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now = ( Gimbal_Auto_Shoot.Armor_yaw );																			//±¾´Î²É¼¯µÄ½Ç¶È
//                Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now;
//                Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now = Gimbal_Auto_Shoot.Armor_pit;
//                Gimbal_Auto_Shoot.Speed_Prediction.time1 = Gimbal_Auto_Shoot.Speed_Prediction.time2;																		//ÉÏ´Î²É¼¯µÄÊ±¼ä
//                Gimbal_Auto_Shoot.Speed_Prediction.time2 = time_tick_1ms;																																//±¾´Î²É¼¯µÄÊ±¼ä
//                Gimbal_Auto_Shoot.Speed_Prediction.time_error = Gimbal_Auto_Shoot.Speed_Prediction.time2 - Gimbal_Auto_Shoot.Speed_Prediction.time1;
//                Gimbal_Auto_Shoot.Speed_Prediction.yaw_angle_error = Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now - Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Pre;
//                Gimbal_Auto_Shoot.Speed_Prediction.pit_angle_error = Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now - Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Pre;
//               
//                Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw = ( ( Gimbal_Auto_Shoot.Speed_Prediction.yaw_angle_error )						//½ÇËÙ¶È¼ÆËã
//                        / ( Gimbal_Auto_Shoot.Speed_Prediction.time_error ) ) * 1000;
//                Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Pit = ( ( Gimbal_Auto_Shoot.Speed_Prediction.pit_angle_error )
//                        / ( Gimbal_Auto_Shoot.Speed_Prediction.time_error ) ) * 1000;


////                if ( fabs ( Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw ) < 50 )
//                if ( fabs ( Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw ) < 100 )
//                {
//                    Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed;		//ÉÏ´ÎµÄ½ÇËÙ¶È
//                    Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed = Gimbal_Auto_Shoot.Speed_Prediction.Army_Speed_Yaw;						//±¾´ÎµÄ½ÇËÙ¶È
//                    Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Acceleration = ( ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed			//½Ç¼ÓËÙ¶È¼ÆËã(Î´Ê¹ÓÃ)
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


//        //-/---------------- ¿¨¶ûÂüÂË²¨²¢¸üÐÂÔÆÌ¨¸ø¶¨½Ç¶È----------------------------/-//
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

//        //-/-------------------- ¼ÆËãÍ¼Ïñ´¦ÀíºÍÔÆÌ¨¶¯×÷ÑÓ³ÙÁ¿£¬×÷ÎªÔ¤²âµÄ²¹³¥ ----------------------/-//
//#if ARMY_SPEED_PREDICTION == 1
//        if ( Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag == 1 )
//        {
//            Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag = 0;
//#if ENABLE_KALMAN_FILTER == 1	           //Ê¹ÓÃ¿¨¶ûÂüÂË²¨Ö®ºóµÄËÙ¶È½øÐÐ²¹³¥£¬²¹³¥Í¼Ïñ´¦ÀíÑÓ³ÙºÍµ¯µÀÑÓ³Ù
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



//#else							                       //Ê¹ÓÃÖ±½ÓÇóµÃµÄËÙ¶È½øÐÐ²¹³¥	
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
//        //Æ«²î½Ç´óÓÚ2¡ã£¬·ÖÎªÁíÖÖÇé¿ö¡£
//        //1¡¢¸ÕÊ¶±ðµ½£¬Æ«²î½Ï´ó£¬Èç¹ûÖ±½Ó²¹³¥»áµ¼ÖÂÔÆÌ¨¶¶¶¯Õðµ´£»
//        //2¡¢Ê¶±ðµ½Ò»¶¨Ê±¼ä£¬µ«ÊÇÓÉÓÚÎÞ²¹³¥µ¼ÖÂ¸ú×ÙÖÍºó
//        //3¡¢ÔÆÌ¨ÔÚ¶¶¶¯£¬µ¼ÖÂyaw·´À¡ÔÚÖÐ¼äÌøÔ¾
//        else
//        {
//            Gimbal_Auto_Shoot.Continue_Large_Err_Cnt++;
//            if ( Gimbal_Auto_Shoot.Continue_Large_Err_Cnt >= 40 )  //300ms£¬³ÖÐø300msµÄÆ«²î£¬±íÃ÷´ËÊ±Îª¸ú×ÙÖÍºó£¬Ðè¼ÓÈë²¹³¥
//            {
//                Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 40;
//                gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
//                gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
//            }
//        }


//#else
//				
//				  //-/------------------------ ¼ÓÈë²¹³¥£¬¸üÐÂÔÆÌ¨¸ø¶¨½Ç¶È ----------------------/-//

//          //¸ù¾ÝÆ«²î½Ç¼ÆËãÆ«²î¾àÀë£¬ÓÉÓÚ¾àÀëÐÅÏ¢´æÔÚÎó²î£¬²»Ê¹ÓÃ¼ÆËãµÄtemp = fabs(Gimbal_Auto_Shoot.Distance * 2.0f * arm_sin_f32(0.5f * fabs(Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw) * 0.01745329252f ));
//          //Æ«²î½Ç¶ÈÐ¡ÓÚ2¶È£¬ËµÃ÷´ËÊ±»ù±¾Ãé×¼×°¼×Æ¬£¬Á½¸öÊ±¼ä²ÎÊýÓ¦¸ÃÐèÒªµ÷Õû¡£
//          //¸Ã´¦Ó¦¸Ã¼ÓÉÏpitchÖáµÄÔ¤²âÅÐ¶¨
//        if ( fabs ( Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw ) < 3.0 + fabs ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation ) )
//        {
//            Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 0;
//            gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
//            gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
//        }
//        //Æ«²î½Ç´óÓÚ2¡ã£¬·ÖÎªÁíÖÖÇé¿ö¡£
//        //1¡¢¸ÕÊ¶±ðµ½£¬Æ«²î½Ï´ó£¬Èç¹ûÖ±½Ó²¹³¥»áµ¼ÖÂÔÆÌ¨¶¶¶¯Õðµ´£»
//        //2¡¢Ê¶±ðµ½Ò»¶¨Ê±¼ä£¬µ«ÊÇÓÉÓÚÎÞ²¹³¥µ¼ÖÂ¸ú×ÙÖÍºó
//        //3¡¢ÔÆÌ¨ÔÚ¶¶¶¯£¬µ¼ÖÂyaw·´À¡ÔÚÖÐ¼äÌøÔ¾
//        else
//        {
//            Gimbal_Auto_Shoot.Continue_Large_Err_Cnt++;
//            if ( Gimbal_Auto_Shoot.Continue_Large_Err_Cnt >=100 )  //300ms£¬³ÖÐø300msµÄÆ«²î£¬±íÃ÷´ËÊ±Îª¸ú×ÙÖÍºó£¬Ðè¼ÓÈë²¹³¥
//            {
//                Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 100;
//                gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
//                gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
//            }
//        }

//#endif
//        Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation = 0;         //²¹³¥Ò»´ÎºóÇåÁã
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
//        gim.pid.yaw_angle_fdb =  yaw_Angle;//-GMYawEncoder.ecd_angle;//×îÖÕ×ª»»³É½Ç¶È  ÏµÊýÐèÒªµ÷Õû
//        gim.pid.pit_angle_fdb =  pitch_Angle;//GMPitchEncoder.ecd_angle;//×îÖÕ×ª»»³É½Ç¶È  ÏµÊýÐèÒªµ÷Õû

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

    /*×Ô¶¯´ò»÷·¢Éä¿ØÖÆÂß¼­
    	1.ÓÉÓÚÎÞ·¨×°ÏÞÎ»¿ª¹Ø£¬ÐèÏÈ´ò¿ªÄ¦²ÁÂÖÔÙ´ò¿ª²¦ÅÌ£¬²»¿É±ÜÃâ»áÔì³ÉÒ»¶¨³Ì¶ÈÉÏµÄÑÓÊ±£¬³õ²½Îª100ms
    	2.¹Ø±Õ·¢ÉäÊ±Ó¦Í¬Ê±¹Ø±Õ²¦ÅÌºÍÄ¦²ÁÂÖ
    	3.×°¼×½øÈëÒ»¶¨ÇøÓòÊ±´¥·¢·¢Éä×´Ì¬AAA
    	4.ÐèÒªºÍÒ£¿ØÆ÷×´Ì¬ÎÞ·ìÏÎ½Ó±ãÓÚ¿ØÖÆ
    	5.Ðè²¦ÖÁÌØ¶¨µµÎ»²Å¿ªÆôÉä»÷¹¦ÄÜ
      6.¹Ø±ÕÖ®ºóÉÏµ¯ÖÁ¶Â×ª


    */
    switch ( friction_wheel_state )
    {
        case FRICTION_WHEEL_OFF:
        {
            frictionRamp.ResetCounter ( &frictionRamp );
            if ( refromData.s1 != 3  &&  ( new_location.recogflag != 0 ) ) //´Ó¹Ø±Õµ½start turning
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
            if ( auto_mode == AUTO_PATROL || lostflag == 1 ) //¸ÕÆô¶¯¾Í±»¹Ø±Õ
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
                /*Ô¤°¸£¬ÏÞÎ»¿ª¹Ø²»ºÃÊ¹¾ÍÓÃÕâ¸ö*/

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

