#include  "main.h"
chassis_t chassis;
position_e robot_position;			
direction_e robot_direction;		
sensor_state_e sensor_l = sensor_off;
sensor_state_e sensor_r = sensor_off;
uint8_t    crazyflag = 1;
uint8_t		 crazyspeeddir =0;
int 			 crazyspeed=0;
int 			 crazytime=0;
Power_Control_Struct Power_Control = POWER_CONTROL_DEFAULT;



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

		/*   检测两侧圆柱  */
		if ( GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_0 ) == 0 )
    {
      sensor_l = sensor_on;
    }
		else if ( GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_0 ) == 1 )
		{
			sensor_l = sensor_off;
		}
    if ( GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_1 ) == 0 )
    {
			sensor_r = sensor_on;
		}
    else if ( GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_1 ) == 1 )
    {
      sensor_r = sensor_off;
    }
		
		/* 放在最后 确保不撞柱子 */
		if(chassis.ctrl_mode == CHASSIS_PATROL)					//巡逻模式变向
		{
			if (  sensor_r == sensor_on && chassis.vx > 0 )
			{
				if(crazyflag == 1)
				{
					crazyspeed = -crazyspeed;
				}
				else
					chassis.vx = -chassis.vx;
			}
			if (  sensor_l == sensor_on && chassis.vx < 0 )
			{
				 if(crazyflag == 1)
				{
					crazyspeed = -crazyspeed;
				}
				else
					chassis.vx = -chassis.vx;
			}
		}
		else if(chassis.ctrl_mode == CHASSIS_REMOTE)		//遥控器模式停车
		{
			if (  sensor_r == sensor_on && chassis.vx > 0 )
			{
				chassis.vx = 0;
			}
			if (  sensor_l == sensor_on && chassis.vx < 0 )
			{
				chassis.vx = 0;
			}
		}

		chassis.wheel_speed_ref = chassis.vx;						//vx>0向右,vx<0向左
		chassis.wheel_speed_fdb=CM1Encoder.filter_rate;

		chassis.current = pid_calc(&pid_spd, chassis.wheel_speed_fdb, chassis.wheel_speed_ref);
		
		power_limit_handle();
		if(chassis.ctrl_mode != CHASSIS_RELAX)
		{
			CAN1_Send_Msg(CAN1,0,0,chassis.current,0);
		}
		else
		{
			CAN1_Send_Msg(CAN1,0,0,0,0);
		}
    

}

void chassis_remote_handle(void)
{
  chassis.vx = ChassisSpeedRef.forward_back_ref;
}



void chassis_patrol_handle(void)
{
    switch ( robot_direction )
    {
        case direction_left:    //向左运动
        {
						chassis.vx = -600;
						if(sensor_l == sensor_on)						//变向
						{
							robot_direction = direction_right;
						}
        }
        break;
        case direction_right:    //向右运动
        {
						chassis.vx = 600;
						if(sensor_r == sensor_on)						//变向
						{
							robot_direction = direction_left;
						}
        }
        break;
        case direction_stop:   //停止
        {
            chassis.vx = 0 ;
        }
        break;
        default:
				{
					chassis.vx = 0;
				}
        break;
    }

		if(crazyflag)
		{
			if(crazytime == 0)
			{
				crazyspeeddir = rand()%2;
				if(crazyspeeddir == 1)
				{
					crazyspeed = rand()%150 + 400;
				}
				else if(crazyspeeddir == 0)
				{
					crazyspeed = -(rand()%150 +400);
				}
				crazytime  = rand()%150 + 50;
			}
			chassis.vx = crazyspeed;
			crazytime--;
		}

}

void chassis_stop_handle(void)
{
  chassis.vy =0;
  chassis.vx =0;
  pid_clr(&pid_spd);

}


/**
  * @brief  nitialize chassis motor pid parameter
  * @usage  before chassis loop use this function
**/
void chassis_param_init(void)//底盘参数初始化
{
  memset(&chassis, 0, sizeof(chassis_t));			
  chassis.ctrl_mode      = CHASSIS_REMOTE;
  chassis.last_ctrl_mode = CHASSIS_RELAX;
	
	robot_direction = direction_right;
  PID_struct_init ( &pid_spd, POSITION_PID, 12000, 3000, 45.0f, 0, 0 );
}

/**
  * @brief  chassis power limit
  * @usage  set Max_Power
**/
int32_t total_cur_limit;
int32_t total_cur;
u8  Max_Power     = 30;
float I_TIMES_V_TO_WATT =   0.0000189f ;
float power_limit_rate = 1;
void power_limit_handle ( void )
{
#if POWER_LIMIT_MODE==0
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
#elif POWER_LIMIT_MODE==1
    if ( judge_rece_mesg.power_heat_data.chassis_power_buffer < WARNING_ENERGY )
        total_cur_limit = ( ( float ) judge_rece_mesg.power_heat_data.chassis_power_buffer / WARNING_ENERGY );
    else
        total_cur_limit = 1;


    float drive_power = ( i_predict ( 0, 1.0f ) * ( float ) chassis.wheel_speed_fdb  ) * I_TIMES_V_TO_WATT;

    float heat_power = heat_power_calc ( i_predict ( 0, 1.0f ) );


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
        //	i_n=a[n]*k+b[n]	带入
        //Max_Power=m*k^2+n*k+o
        //0=m*k^2+n*k+l(l=o-Max_Power)

        float m = ( a * a ) * FACTOR_2;

        float n = 2 * FACTOR_2 * ( a * b ) + \
                  FACTOR_1 * ( a ) + \
                  I_TIMES_V_TO_WATT * ( a * ( float ) chassis.wheel_speed_fdb );

        float l = ( b * b ) * FACTOR_2 + \
                  ( b ) * FACTOR_1 + \
                  I_TIMES_V_TO_WATT * ( b * ( float ) chassis.wheel_speed_fdb ) + FACTOR_0 - Max_Power;

        //一元二次求根公式
        power_limit_rate = ( -n + ( float ) sqrt ( ( double ) ( n * n - 4 * m * l ) ) ) / ( 2 * m ) * total_cur_limit;

        //没有考虑热功率
//							 power_limit_rate=1.0f -
//														  (float)(PowerSum - Max_Power) /
//															(pid_spd[0].p* ((float)chassis.wheel_speed_ref[0]*(float)chassis.wheel_speed_fdb[0] + \
//																						  (float)chassis.wheel_speed_ref[1]*(float)chassis.wheel_speed_fdb[1] + \
//																						  (float)chassis.wheel_speed_ref[2]*(float)chassis.wheel_speed_fdb[2] + \
//																						  (float)chassis.wheel_speed_ref[3]*(float)chassis.wheel_speed_fdb[3])) \
//														/I_TIMES_V_TO_WATT;

        VAL_LIMIT ( power_limit_rate, 0, 1 );
    }
    else
    {
        power_limit_rate = 1;
    }
#endif
}

