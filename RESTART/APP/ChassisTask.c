#include  "main.h"
chassis_t chassis;

sensor_state_e sensor_l = sensor_off;
sensor_state_e sensor_r = sensor_off;
Power_Control_Struct Power_Control = POWER_CONTROL_DEFAULT;
int last_remain_HP = 0;

void chassis_task(void)
{
		//40ms �ж�һ���Ƿ��յ�����ϵͳ����
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
			case CHASSIS_REMOTE:  			//ң�������˿���
			{
				chassis_remote_handle();
			}
			break;
			case CHASSIS_PATROL:				//Ѳ��ģʽ
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

		/*   �������Բ��  */
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
		
		
		chassis.wheel_speed_ref = chassis.vx ;
		chassis.wheel_speed_fdb = CM1Encoder.filter_rate;

		if( chassis.powerlimit == limit_strict)
		{
			power_limit_handle();		//��ǰԤ���ϸ����ƹ���
			chassis.current = pid_calc(&pid_spd, chassis.wheel_speed_fdb, chassis.wheel_speed_ref*power_limit_rate);
		}
		else if (chassis.powerlimit == limit_buffer)
		{
			chassis.current = pid_calc(&pid_spd, chassis.wheel_speed_fdb, chassis.wheel_speed_ref);
			power_limit_handle();			//ʹ�û���������������
		}

		if(chassis.ctrl_mode != CHASSIS_RELAX)
		{
			CAN1_Send_Msg(CAN1,0,0,chassis.current,0);
		}
		else
		{
			CAN1_Send_Msg(CAN1,0,0,0,0);
		}
    

}

void chassis_remote_handle(void)		//ң��������
{
		chassis.powerlimit = limit_strict;
		chassis.vx = ChassisSpeedRef.forward_back_ref;
		if ( (sensor_r == sensor_on && chassis.vx > 0) || (sensor_l == sensor_on && chassis.vx < 0) )	//����ֹײ��
		{
				chassis.vx = 0;
		}
}


void chassis_patrol_handle(void)		//Ѳ��
{
		if(judge_rece_mesg.game_robot_state.remain_HP < last_remain_HP)
		{
			chassis.crazydata.crazyflag = 1;
			chassis.crazydata.crazytime = 200;
		}
	//����ģʽ
		if(chassis.crazydata.crazyflag)
		{
			if(judge_rece_mesg.power_heat_data.chassis_power_buffer > WARNING_ENERGY)
			chassis.powerlimit = limit_buffer;
		else
			chassis.powerlimit = limit_strict;
			if(chassis.crazydata.crazychangetime == 0)
			{
				chassis.crazydata.crazyspeed			= rand()%150 + 500;
				chassis.crazydata.crazychangetime		  	= rand()%70 + 70;

#if			CRAZY_DIR_CHANGE_MODE == 0
				chassis.direction = !chassis.direction;
				
#elif			CRAZY_DIR_CHANGE_MODE == 1
				chassis.crazydata.crazyspeeddir 	= rand()%2;
				if(chassis.crazydata.crazyspeeddir == 0)
						chassis.direction = direction_right;
				else if(chassis.crazydata.crazyspeeddir == 1)
						chassis.direction = direction_left;
#endif

			}
			chassis.crazydata.crazychangetime--;
			chassis.crazydata.crazytime--;
			if(chassis.crazydata.crazytime <= 0)
			{
				chassis.crazydata.crazyflag = 0;
				chassis.powerlimit = limit_strict;
			}
		}
		
		//����ֹײ��
		if(sensor_l == sensor_on && chassis.direction == direction_left)						//����
		{
			chassis.direction = direction_right;
		}
		else if(sensor_r == sensor_on && chassis.direction == direction_right)			//����
		{
			chassis.direction = direction_left;
		}
		

    switch ( chassis.direction )
    {
        case direction_left:    //�����˶�
        {
					if(chassis.crazydata.crazyflag)
						chassis.vx = -chassis.crazydata.crazyspeed;
					else
						chassis.vx = -500;
        }
        break;
				
        case direction_right:    //�����˶�
        {
					if(chassis.crazydata.crazyflag)
						chassis.vx = chassis.crazydata.crazyspeed;
					else
						chassis.vx = 500;
        }
        break;
				
        case direction_stop:   //ֹͣ
        {
            chassis.vx = 0 ;
        }
        break;
        default:
        break;
    }
		
		last_remain_HP = judge_rece_mesg.game_robot_state.remain_HP;
}

void chassis_stop_handle(void)			//ͣ��
{
  chassis.vx =0;
  pid_clr(&pid_spd);
}

void chassis_param_init(void)//���̲�����ʼ��
{
  memset(&chassis, 0, sizeof(chassis_t));			
  chassis.ctrl_mode      = CHASSIS_REMOTE;
  chassis.last_ctrl_mode = CHASSIS_RELAX;
	chassis.crazydata.crazyflag = 0;
	chassis.direction = direction_right;
	chassis.powerlimit = limit_strict;
  PID_struct_init ( &pid_spd, POSITION_PID, 12000, 3000, 45.0f, 0, 0 );
}


/**
  * @brief  chassis power limit
  * @usage  set Max_Power
**/
static float i_torque(float factor)		//ת�ص�������
{
  float i_torque= pid_spd.p * (factor*(float)chassis.wheel_speed_ref - chassis.wheel_speed_fdb)+ \
               pid_spd.iout+ \
               pid_spd.d * ((factor*(float)chassis.wheel_speed_ref-chassis.wheel_speed_fdb) - pid_spd.err[LAST]);
  return i_torque;
}
static float heat_power_calc(int i)		//����ȹ���
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

					//���м����i_n=a[n]*k+b[n]
					float a;
					a = ( float ) chassis.wheel_speed_ref * ( pid_spd.p + pid_spd.d );
					float b;
					b = -pid_spd.p * ( float ) chassis.wheel_speed_fdb + pid_spd.iout \
							-pid_spd.d * ( float ) chassis.wheel_speed_fdb - pid_spd.d * pid_spd.err[LAST];
					// Max_power=heat_power+drive_power
					//	i_n=a[n]*k+b[n]	����
					//Max_Power=m*k^2+n*k+o
					//0=m*k^2+n*k+l(l=o-Max_Power)

					float m = ( a * a ) * FACTOR_2;

					float n = 2 * FACTOR_2 * ( a * b ) + \
										FACTOR_1 * ( a ) + \
										I_TIMES_V_TO_WATT * ( a * ( float ) chassis.wheel_speed_fdb );

					float l = ( b * b ) * FACTOR_2 + \
										( b ) * FACTOR_1 + \
										I_TIMES_V_TO_WATT * ( b * ( float ) chassis.wheel_speed_fdb ) + FACTOR_0 - Max_Power;

					//һԪ���������ʽ
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

