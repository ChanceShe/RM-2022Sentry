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
    case MANUAL_FOLLOW_GIMBAL:  //跟随云台模式
    {
      follow_gimbal_handle();
    }
		break;
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

void follow_gimbal_handle(void)
{
  chassis.vy = 0.75*ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;
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
  chassis.ctrl_mode      = MANUAL_FOLLOW_GIMBAL;
  chassis.last_ctrl_mode = CHASSIS_RELAX;

  chassis.position_ref = 0;
//  chassis_rotate_flag = 0;

    PID_struct_init ( &pid_spd, POSITION_PID, 12000, 3000, 45.0f, 0, 0 );
//  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 70, 5.0f, 0.01,30.0f);//xisanhao
}
