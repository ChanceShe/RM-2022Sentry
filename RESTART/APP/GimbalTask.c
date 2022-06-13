#include "main.h"
#include "AHRS_MiddleWare.h"		//大疆姿态解算

gimbal_t gim;
float Init_Yaw_Angle  =  0;
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;
float GMYawAngle = 0,GMYawLastAngle = 0;
float GMYawGyro = 0;
float GMPitAngle = 0,GMPitLastAngle = 0;
float GMPitGyro = 0;

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
	 
	 testnum1 = gim.pid.yaw_angle_fdb - gim.pid.yaw_angle_ref ;
	 testnum2 = gim.pid.pit_angle_fdb - gim.pid.pit_angle_ref ;
	 
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
                      10, 0.02, 10 );
		PID_struct_init ( &pid_yaw_speed, POSITION_PID , 28000, 3000,
                      180.0f, 0, 5 );
	//斜坡初始化
    GMPitchRamp.SetScale ( &GMPitchRamp, PREPARE_TIME_TICK_MS );
    GMYawRamp.SetScale ( &GMYawRamp, PREPARE_TIME_TICK_MS );
    GMPitchRamp.ResetCounter ( &GMPitchRamp );
    GMYawRamp.ResetCounter ( &GMYawRamp );
}

void cascade_pid_ctrl ( void )	//级联pid函数
{
		GMYawLastAngle = GMYawAngle;
		GMYawAngle =  GMYawEncoder.ecd_angle;
		GMYawGyro = (GMYawAngle - GMYawLastAngle)/(57.3*0.005);
		GMPitLastAngle = GMPitAngle;
		GMPitAngle =  GMPitchEncoder.ecd_angle;
		GMPitGyro = (GMPitAngle - GMPitLastAngle)/(57.3*0.005);
    gim.pid.yaw_speed_ref = pid_yaw.out;
    gim.pid.pit_speed_ref = pid_pit.out;
    gim.pid.yaw_speed_fdb = GMYawGyro;     //  角速度
    gim.pid.pit_speed_fdb = GMPitGyro;
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
					gim.pid.yaw_angle_ref = gim.pid.yaw_angle_ref + 0.5;

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

//				if(fabs(-(new_location.pitch - PITCH_MIN) - Gimbal_Auto_Shoot.target_pit)<=20)
						Gimbal_Auto_Shoot.target_pit = -(new_location.pitch - PITCH_ZERO);
//				else
//						Gimbal_Auto_Shoot.target_pit = 0;
				
        Gimbal_Auto_Shoot.target_yaw	= new_location.yaw;
    }
    else
    {
        Gimbal_Auto_Shoot.Recognized_Timer++;
        if ( Gimbal_Auto_Shoot.Recognized_Timer == 50 ) //200ms，时间太长，云台保持原有给定导致乱动作
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
						if(Gimbal_Auto_Shoot.target_pit!=0 && Gimbal_Auto_Shoot.target_yaw!= 0)
						{
//								if(Gimbal_Auto_Shoot.target_pit<=PITCH_HIGHLAND)
//								{
//										Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation = -7;//环形高地上
//								}
//								else
//								{
//										Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation = (Gimbal_Auto_Shoot.target_pit - Init_Pitch_Angle)*(3)-3;//环形高地下
//								}
								Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation =  -1.8;
								gim.pid.yaw_angle_ref = Gimbal_Auto_Shoot.target_yaw + Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation;
								gim.pid.pit_angle_ref = Gimbal_Auto_Shoot.target_pit + Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation;
						}
        }


     
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
        }

        Gimbal_Auto_Shoot.Continue_Recognized_Cnt = 0;

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

    if (  gim.pid.pit_angle_fdb < 20.0f + gim.pid.pit_angle_ref	\
            && gim.pid.pit_angle_fdb > -20.0f + gim.pid.pit_angle_ref )
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
//            if ( refromData.s1 != 3  &&  ( new_location.recogflag != 0 ) && ( refromData.JudgeShootFlag == 1 )) //从关闭到start turning
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
//            else if (  ( dir_pitch == 1 ) &&  ( dir_yaw == 1 ) && ( new_location.recogflag != 0 ) )
						  else if (  ( dir_yaw == 1 ) && ( new_location.recogflag != 0 ) )
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

