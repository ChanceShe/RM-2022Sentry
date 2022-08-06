#include  "main.h"
chassis_t chassis;
uint8_t brake_en;
uint32_t chassis_patrol_time = 0;
Power_Control_Struct Power_Control = POWER_CONTROL_DEFAULT;
int last_remain_HP = 0;

void chassis_param_init(void)//底盘参数初始化
{
  memset(&chassis, 0, sizeof(chassis_t));
  chassis.ctrl_mode      = CHASSIS_REMOTE;
  chassis.last_ctrl_mode = CHASSIS_RELAX;
	chassis.crazydata.crazyflag = 0;
	chassis.direction = direction_right;
	
#if			POWER_LIMIT_MODE == 0
	chassis.powerlimit = limit_strict;
#elif			POWER_LIMIT_MODE == 1
	chassis.powerlimit = limit_buffer;
#endif
	
  PID_struct_init ( &pid_spd, POSITION_PID, 12000, 3000, 45.0f, 0, 0 );
	PID_struct_init ( &pid_brake, POSITION_PID, 10000,10000,60 , 0, 0 );
	patrolrange_init();
//	  GPIO_InitTypeDef gpio;
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
//	  gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
//    gpio.GPIO_Mode = GPIO_Mode_IN;
//    gpio.GPIO_Speed = GPIO_Speed_50MHz;
//    gpio.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//    GPIO_Init(GPIOC, &gpio);
//		GPIO_SetBits(GPIOC,GPIO_Pin_8);
//		GPIO_SetBits(GPIOC,GPIO_Pin_9);
}

void chassis_task(void)
{
		//40ms 判断一次是否收到裁判系统数据
		Power_Control.Time_10ms++;
    if ( Power_Control.Time_10ms > 4 )	
    {
			Power_Control.Time_10ms = 0;
			if ( Power_Control.Cnt_Power_Judge_Recieved - Power_Control.Cnt_Power_Judge_Recieved_Pre == 0 )
					Power_Control.Flag_Judge_Control = 0;
			else
					Power_Control.Flag_Judge_Control = 1;
			Power_Control.Cnt_Power_Judge_Recieved_Pre = Power_Control.Cnt_Power_Judge_Recieved;
		}
		
		switch (chassis.ctrl_mode)
    {
			case CHASSIS_STOP:
			{
				chassis_stop_handle();
			}
			break;
			case CHASSIS_REMOTE:  			//遥控器拨杆控制
			{
				chassis_remote_handle();
			}
			break;
			case CHASSIS_PATROL:				//巡逻模式
			{
				chassis_patrol_handle();
			}
			break;
			case CHASSIS_RELAX:
			{
			}
			break;
			default:
			{
				chassis_stop_handle();
			}
			break;
		}

//		/*   检测两侧圆柱  */
//		if ( GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_8 ) == 0 )
//    {
//      chassis.opt_switch_l = switch_on;
//    }
//		else if ( GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_8 ) == 1 )
//		{
//			chassis.opt_switch_l = switch_off;
//		}
//    if ( GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_9 ) == 0 )
//    {
//			chassis.opt_switch_r = switch_on;
//		}
//    else if ( GPIO_ReadInputDataBit ( GPIOC, GPIO_Pin_9 ) == 1 )
//    {
//      chassis.opt_switch_r = switch_off;
//    }
//		GPIO_ResetBits(GPIOC,GPIO_Pin_8);
//		GPIO_ResetBits(GPIOC,GPIO_Pin_9);
		
		chassis.wheel_speed_ref = chassis.vx ;
		chassis.wheel_speed_fdb = CM1Encoder.filter_rate;

		if( chassis.powerlimit == limit_strict)
		{
			power_limit_handle();		//超前预测严格限制功率
			chassis.current = pid_calc(&pid_spd, chassis.wheel_speed_fdb, chassis.wheel_speed_ref*power_limit_rate);
		}
		else if (chassis.powerlimit == limit_buffer)
		{
			chassis.current = pid_calc(&pid_spd, chassis.wheel_speed_fdb, chassis.wheel_speed_ref);
			power_limit_handle();			//使用缓冲能量功率限制
		}

		if(chassis.ctrl_mode == CHASSIS_RELAX || brake_en)
		{
			CAN1_Send_Msg(CAN1,0,0,0,0);
		}
		else
		{
			CAN1_Send_Msg(CAN1,0,0,chassis.current,0);
		}
    

}

void chassis_remote_handle(void)		//遥控器控制
{
		chassis.powerlimit = limit_buffer;
		chassis.vx = ChassisSpeedRef.forward_back_ref;
//		if ( (chassis.opt_switch_r == sensor_on && chassis.vx > 0) || (chassis.opt_switch_l == sensor_on && chassis.vx < 0) )	//检测防止撞柱
//		{
//				chassis.vx = 0;
//		}
}

uint16_t dis_sta= 0,dis_end = 0;				//随机运动区间起终点距立柱距离
uint8_t sta_sec,end_sec;								//起始，终止段号
uint8_t changedir_count = 0;						//变向计数
void patrolrange_init(void)
{
	sta_sec = rand()%9+3;
	end_sec = sta_sec+(rand()%3+3)-1;
	dis_sta = (int)(DIS_L/15)*(sta_sec-1);		//起点距左立柱距离
	dis_end = (int)(DIS_R/15)*(15-end_sec);		//终点距右立柱距离
	changedir_count = 3;
	if(TF02_L.Dist - dis_sta >= TF02_R.Dist - dis_end)
	{
		if(chassis.direction == direction_right)
			brake_en = 1;
	}
	else
	{
		if(chassis.direction == direction_left)
			brake_en = 1;
	}
	chassis.speed = rand()%100+500;
}
void chassis_patrol_handle(void)		//巡逻
{
//			if(judge_rece_mesg.game_robot_state.remain_HP < last_remain_HP)
//				{
//					chassis.crazydata.crazyflag = 1;
//					chassis.crazydata.crazytime = 1000;
//				}
			if((chassis.direction == direction_left&&TF02_L.Dist < dis_sta)||(chassis.direction == direction_right&&TF02_R.Dist < dis_end))
			{
				changedir_count--;
				if(changedir_count == 0)
					patrolrange_init();
				else
				{
					brake_en = 1;
					chassis.speed = rand()%100+500;
				}
			}
//		//暴走模式
//			if(chassis.crazydata.crazyflag)
//			{
//				if(judge_rece_mesg.power_heat_data.chassis_power_buffer > WARNING_ENERGY)
//					chassis.powerlimit = limit_buffer;
//				else
//					chassis.powerlimit = limit_strict;
//				if(chassis.crazydata.crazychangetime == 0&&chassis.powerlimit == limit_buffer)
//				{
//					chassis.crazydata.crazyspeed			= rand()%100 + 600;
//					chassis.crazydata.crazychangetime		  	= rand()%100 + 100;

//	#if			CRAZY_DIR_CHANGE_MODE == 0
//					#if BRAKE_EN == 0
//					chassis.direction = !chassis.direction;
//					#elif BRAKE_EN == 1
//					brake_en = 1;
//					#endif
//	#elif			CRAZY_DIR_CHANGE_MODE == 1
//					chassis.crazydata.crazyspeeddir 	= rand()%10;
//					if(chassis.crazydata.crazyspeeddir >= 3)
//							#if BRAKE_EN == 0
//							chassis.direction = !chassis.direction;
//							#elif BRAKE_EN == 1
//							brake_en = 1;
//							#endif				
//					else if(chassis.crazydata.crazyspeeddir < 3)
//							chassis.direction = chassis.direction;
//	#endif

//				}
//				else if(chassis.crazydata.crazychangetime == 0 && chassis.powerlimit == limit_strict)
//				{
//					chassis.crazydata.crazyspeed			= rand()%100 + 500;
//					chassis.crazydata.crazychangetime		  	= rand()%70 + 200;

//	#if			CRAZY_DIR_CHANGE_MODE == 0		//被击打即变向
//					#if BRAKE_EN == 0
//					chassis.direction = !chassis.direction;
//					#elif BRAKE_EN == 1
//					brake_en = 1;
//					#endif
//	#elif			CRAZY_DIR_CHANGE_MODE == 1
//					chassis.crazydata.crazyspeeddir 	= rand()%10;
//					if(chassis.crazydata.crazyspeeddir >= 3)
//							#if BRAKE_EN == 0
//							chassis.direction = !chassis.direction;
//							#elif BRAKE_EN == 1
//							brake_en = 1;
//							#endif				
//					else if(chassis.crazydata.crazyspeeddir < 3)
//							chassis.direction = chassis.direction;
//	#endif

//				}
//				chassis.crazydata.crazychangetime--;
//				chassis.crazydata.crazytime--;
//				if(chassis.crazydata.crazytime <= 0)
//				{
//					chassis.crazydata.crazyflag = 0;
//				}
//			}
//			else
//			{
//					#if			POWER_LIMIT_MODE == 0
//						chassis.powerlimit = limit_strict;
//					#elif			POWER_LIMIT_MODE == 1
//						chassis.powerlimit = limit_buffer;
//					#endif
//			}
//			
//			if((TF02_L.Dist<50&&TF02_L.Dist>5) && chassis.direction == direction_left)						//变向
//			{
//				#if BRAKE_EN == 0
//					chassis.direction = direction_right;
//				#elif BRAKE_EN == 1
//					brake_en = 1;
//				#endif	
//			}
//			else if((TF02_R.Dist<50&&TF02_R.Dist>5) && chassis.direction == direction_right)			//变向
//			{
//				#if BRAKE_EN == 0
//					chassis.direction = direction_left;
//				#elif BRAKE_EN == 1
//					brake_en = 1;
//				#endif	
//			}
		if(brake_en)
		{
			chassis.vx = 0;
			ShootFlag = 0;
			if(chassis.direction == direction_left)
			{
				pid_brake.get = BrakeEncoder.filter_rate;
				pid_brake.set = -300;
				if(CM1Encoder.filter_rate>=-5)
				{
					chassis.direction = direction_right;
					brake_en = 0;
					chassis_patrol_time = 0;
				}
			}
			else if(chassis.direction == direction_right)
			{
				pid_brake.get = BrakeEncoder.filter_rate;
				pid_brake.set = 300;
				if(CM1Encoder.filter_rate<=5)
				{
					chassis.direction = direction_left;
					brake_en = 0;
					chassis_patrol_time = 0;
				}
			}

			
			pid_calc ( &pid_brake, pid_brake.get, pid_brake.set );
			CAN1_Send_Msg ( CAN1, 0, 0, 0, pid_brake.out );
			
		}
		else
		{
			ShootFlag = 1;
			chassis_patrol_time++;
				switch ( chassis.direction )
				{
						case direction_left:    //向左运动
						{
							if(chassis.crazydata.crazyflag)
								chassis.vx = -chassis.crazydata.crazyspeed;
							else
								chassis.vx = -chassis.speed;
						}
						break;
						case direction_right:    //向右运动
						{
							if(chassis.crazydata.crazyflag)
								chassis.vx = chassis.crazydata.crazyspeed;
							else
								chassis.vx = chassis.speed ;
						}
						break;
						
						case direction_stop:   //停止
						{
								chassis.vx = 0 ;
						}
						break;
						default:
						break;
				}
				#if BRAKE_EN == 1	
				if(chassis_patrol_time >=100 && fabs((double)chassis.wheel_speed_ref-(double)chassis.wheel_speed_fdb)>500)
				{
						brake_en = 1;
				}
				#endif
				
				CAN1_Send_Msg ( CAN1, 0, 0, 0, 0 );		//给刹车电机卸力
		}
		last_remain_HP = judge_rece_mesg.game_robot_state.remain_HP;
}

void chassis_stop_handle(void)			//停车
{
  chassis.vx =0;
  pid_clr(&pid_spd);
}



/**
  * @brief  chassis power limit
  * @usage  set Max_Power
**/
static float i_torque(float factor)		//转矩电流计算
{
  float i_torque= pid_spd.p * (factor*(float)chassis.wheel_speed_ref - chassis.wheel_speed_fdb)+ \
               pid_spd.iout+ \
               pid_spd.d * ((factor*(float)chassis.wheel_speed_ref-chassis.wheel_speed_fdb) - pid_spd.err[LAST]);
  return i_torque;
}
static float heat_power_calc(int i)		//电机热功率
{
  return  FACTOR_2*(float)(i*i)+FACTOR_1*(float)(i)+FACTOR_0;
}

int32_t total_cur_limit;
int32_t total_cur;
u8  Max_Power     = 25;
float power_limit_rate = 1;
void power_limit_handle ( void )
{
	switch(chassis.powerlimit)
	{
		case(limit_buffer):
		{
			if ( Power_Control.Flag_Judge_Control == 0 )
			{
					//judge system offline, mandatory limit current
					total_cur_limit = 18000;
			}
			else
			{
					if ( judge_rece_mesg.power_heat_data.chassis_power_buffer < WARNING_ENERGY )
							total_cur_limit = ( ( ( float ) judge_rece_mesg.power_heat_data.chassis_power_buffer * \
																		( float ) judge_rece_mesg.power_heat_data.chassis_power_buffer ) / \
																	( WARNING_ENERGY * WARNING_ENERGY ) ) * 24000;
					else
							total_cur_limit = 24000;
			}

			total_cur =  abs ( chassis.current ) ;

			if ( total_cur > total_cur_limit )
			{
					chassis.current = ( float ) chassis.current / ( float ) total_cur * ( float ) total_cur_limit;
			}
		}
		break;
		case(limit_strict):
		{
			if ( judge_rece_mesg.power_heat_data.chassis_power_buffer < WARNING_ENERGY )
					total_cur_limit = ( ( float ) judge_rece_mesg.power_heat_data.chassis_power_buffer / WARNING_ENERGY );
			else
					total_cur_limit = 1;


			float drive_power = ( i_torque ( 1.0f ) * ( float ) chassis.wheel_speed_fdb  ) * I_TIMES_V_TO_WATT;
			float heat_power = heat_power_calc ( i_torque ( 1.0f ) );
			float PowerSum = drive_power + heat_power;


			VAL_LIMIT ( PowerSum, 0, 1000 );
			if ( PowerSum > ( float ) Max_Power )
			{

					//设中间变量i_n=a[n]*k+b[n]
					float a;
					a = ( float ) chassis.wheel_speed_ref * ( pid_spd.p + pid_spd.d );
					float b;
					b = -pid_spd.p * ( float ) chassis.wheel_speed_fdb + pid_spd.iout \
							-pid_spd.d * ( float ) chassis.wheel_speed_fdb - pid_spd.d * pid_spd.err[LAST];
					// Max_power=heat_power+drive_power
					// i_n=a[n]*k+b[n]	带入
					// Max_Power=m*k^2+n*k+o
					// 0=m*k^2+n*k+l(l=o-Max_Power)

					float m = ( a * a ) * FACTOR_2;

					float n = 2 * FACTOR_2 * ( a * b ) + \
										FACTOR_1 * ( a ) + \
										I_TIMES_V_TO_WATT * ( a * ( float ) chassis.wheel_speed_fdb );

					float l = ( b * b ) * FACTOR_2 + \
										( b ) * FACTOR_1 + \
										I_TIMES_V_TO_WATT * ( b * ( float ) chassis.wheel_speed_fdb ) + FACTOR_0 - Max_Power;

					//一元二次求根公式
					power_limit_rate = ( -n + ( float ) sqrt ( ( double ) ( n * n - 4 * m * l ) ) ) / ( 2 * m ) * total_cur_limit;

					VAL_LIMIT ( power_limit_rate, 0, 1 );
			}
			else
			{
					power_limit_rate = 1;
			}
		}
		break;
		case(limit_current):
		{
			if(judge_rece_mesg.power_heat_data.chassis_power_buffer< WARNING_ENERGY)
					total_cur_limit =(judge_rece_mesg.power_heat_data.chassis_power_buffer/WARNING_ENERGY);
			else
					total_cur_limit =1;

				chassis.current = chassis.current * total_cur_limit;
		}
		break;
	}
}


