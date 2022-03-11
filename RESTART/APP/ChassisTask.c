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
		
		/* ������� ȷ����ײ���� */
		if(chassis.ctrl_mode == CHASSIS_PATROL)					//Ѳ��ģʽ����
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
		else if(chassis.ctrl_mode == CHASSIS_REMOTE)		//ң����ģʽͣ��
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

		chassis.wheel_speed_ref = chassis.vx;						//vx>0����,vx<0����
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
        case direction_left:    //�����˶�
        {
						chassis.vx = -600;
						if(sensor_l == sensor_on)						//����
						{
							robot_direction = direction_right;
						}
        }
        break;
        case direction_right:    //�����˶�
        {
						chassis.vx = 600;
						if(sensor_r == sensor_on)						//����
						{
							robot_direction = direction_left;
						}
        }
        break;
        case direction_stop:   //ֹͣ
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
void chassis_param_init(void)//���̲�����ʼ��
{
  memset(&chassis, 0, sizeof(chassis_t));			
  chassis.ctrl_mode      = CHASSIS_REMOTE;
  chassis.last_ctrl_mode = CHASSIS_RELAX;
	
	robot_direction = direction_right;
  PID_struct_init ( &pid_spd, POSITION_PID, 12000, 3000, 45.0f, 0, 0 );
}
