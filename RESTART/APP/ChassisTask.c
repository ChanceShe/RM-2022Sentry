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

void chassis_task(void)
{

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
		case CHASSIS_SEPARATE_GIMBAL:
    {
      chassis.vx = ChassisSpeedRef.forward_back_ref;
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

		CAN2_Send_Msg1(CAN2,chassis.current,0,0,0);

    

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
					crazyspeed = rand()%150 + 500;
				}
				else if(crazyspeeddir == 0)
				{
					crazyspeed = -(rand()%150 + 500);
				}
				crazytime  = rand()%50 + 50;
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
  */
void chassis_param_init(void)//底盘参数初始化
{
  memset(&chassis, 0, sizeof(chassis_t));			
  chassis.ctrl_mode      = CHASSIS_REMOTE;
  chassis.last_ctrl_mode = CHASSIS_RELAX;
	
	robot_direction = direction_right;
  PID_struct_init ( &pid_spd, POSITION_PID, 12000, 3000, 45.0f, 0, 0 );
}
