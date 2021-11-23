#include  "main.h"
chassis_t chassis;


void chassis_task(void)
{

  switch (chassis.ctrl_mode)
    {
    case CHASSIS_STOP:
    {
      chassis_stop_handle();
    }
    break;
    case CHASSIS_CONTROL:  //ң�������˿���
    {
      chassis_control_handle();
    }
		break;
		case CHASSIS_PATROL:				//Ѳ��ģʽ
		{
			chassis_patrol_handle();
		}
    default:
    {
      chassis_stop_handle();
    }
    break;
    }

	chassis.wheel_speed_ref = chassis.vx;
  chassis.wheel_speed_fdb=CM1Encoder.filter_rate;


  chassis.current = pid_calc(&pid_spd, chassis.wheel_speed_fdb, chassis.wheel_speed_ref);

  CAN2_Send_Msg1(CAN2,chassis.current,0,0,0);

    

}

void chassis_control_handle(void)
{
//  chassis.vy = ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;
}


position_e robot_position;
sensor_state_e sensor_l = sensor_off;
sensor_state_e sensor_r = sensor_off;
void chassis_patrol_handle(void)
{
  if ( sensor_l == sensor_off )	 //û��ʶ��
    {

        if ( GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_0 ) == 0 )
        {
            sensor_l = sensor_on;
            robot_position = position_left;
        }
    }
   else
   {

       if ( GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_0 ) == 1 )
       {
           sensor_l = sensor_off;
       }
   }

    if ( sensor_r == sensor_off )	 //û��ʶ��
    {

        if ( GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_1 ) == 0 )
        {

            sensor_r = sensor_on;
            robot_position = position_right;
        }
    }
    else
    {

        if ( GPIO_ReadInputDataBit ( GPIOA, GPIO_Pin_1 ) == 1 )
        {
            sensor_r = sensor_off;
        }
    }

    if ( sensor_r == sensor_off && sensor_l == sensor_off )
    {
        robot_position = position_middle;
    }

//    switch ( robot_position )
//    {
//        case position_middle:    //�м�λ��
//        {

//        }
//        break;
//        case position_left:    //����
//        {
//            chassis.vx = 100;

//        }
//        break;
//        case position_right:      //����
//        {
//            chassis.vx = -100 ;

//        }
//        break;
//        default:
//				{
//					chassis.vx = 0;
//				}
//        break;
//    }





//    /*   �������Բ��   ������� ȷ����ײ����  */
//    if ( ( sensor_r == sensor_on && chassis.vx < 0 ) || ( sensor_l == 1 && sensor_r == 1 && chassis.vx < 0 ) )
//    {   //���⿪�أ�
//        chassis.vx = -chassis.vx ;

//    }
//    if ( ( sensor_l == sensor_on && chassis.vx > 0 ) || ( sensor_l == 1 && sensor_r == 1 && chassis.vx > 0 ) )
//    {
//        chassis.vx = -chassis.vx ;

//    }

//    if ( sensor_r == sensor_on) 
//    {
//        chassis.vx = 0;
//    }
//    else if ( sensor_l == sensor_on) 
//    {
//        chassis.vx = 0;
//    }



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
  chassis.ctrl_mode      = CHASSIS_CONTROL;
  chassis.last_ctrl_mode = CHASSIS_RELAX;

  chassis.position_ref = 0;
//  chassis_rotate_flag = 0;

    PID_struct_init ( &pid_spd, POSITION_PID, 12000, 3000, 45.0f, 0, 0 );
//  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 70, 5.0f, 0.01,30.0f);//xisanhao
}
