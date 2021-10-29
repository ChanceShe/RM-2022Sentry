#include  "main.h"

chassis_t chassis;

void chassis_task(void)
{

  switch (chassis.ctrl_mode)
    {
//    case CHASSIS_STOP:
//    {
//      chassis_stop_handle();
//    }
//    break;

//    case MANUAL_FOLLOW_GIMBAL:  //跟随云台模式
//    {
////      follow_gimbal_handle();
//    }
//    break;
//    case CHASSIS_ROTATE:         //小陀螺
//    {
////      rotate_follow_gimbal_handle();
//    }
//    break;
		 case CHASSIS_SEPARATE:			//底盘分离
    {
      chassis_separate_handle();
    }
    default:
    {
      chassis_stop_handle();
    }
    break;
    }

  mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.wheel_speed_ref);
  chassis.wheel_speed_fdb[0]=CM1Encoder.filter_rate;
  chassis.wheel_speed_fdb[1]=CM2Encoder.filter_rate;
  chassis.wheel_speed_fdb[2]=CM3Encoder.filter_rate;
  chassis.wheel_speed_fdb[3]=CM4Encoder.filter_rate;


  for (int i = 0; i < 4; i++)
    {
      chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_speed_fdb[i], chassis.wheel_speed_ref[i]);
    }

  CAN2_Send_Msg(CAN2, CHASSIS_SPEED_ATTENUATION * chassis.current[0],\
                CHASSIS_SPEED_ATTENUATION * chassis.current[1],\
                CHASSIS_SPEED_ATTENUATION * chassis.current[2],\
                CHASSIS_SPEED_ATTENUATION * chassis.current[3]);

    

}

void chassis_separate_handle(void)
{
  chassis.vy = 0.75*ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;
  chassis.vw=0;
}
void chassis_remove_task(void)
{
	chassis.vx=-ChassisSpeedRef.forward_back_ref;
	chassis.vy=-0.75*ChassisSpeedRef.left_right_ref;
  chassis.vw=0;
//	chassis.vw=pid_calc(&pid_chassis_angle,GMYawEncoder.ecd_angle,chassis.position_ref-180);
}

void chassis_stop_handle(void)
{
  chassis.vy =0;
  chassis.vx =0;
  chassis.vw =0;
  pid_clr(&pid_spd[0]);
  pid_clr(&pid_spd[1]);
  pid_clr(&pid_spd[2]);
  pid_clr(&pid_spd[3]);

}

/**
  * @brief mecanum chassis velocity decomposition
  * @param input : ↑=+vx(mm/s)  ←=+vy(mm/s)  ccw=+vw(deg/s)角度/秒，ccw旋转
  *        output: every wheel speed(rpm)
  * @note  1=FR 2=FL 3=BL 4=BR
  */
void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
  int16_t wheel_rpm[4];
  float   max = 0;
  wheel_rpm[0] = (-vx + vy + vw*5)*2;
  wheel_rpm[1] = ( vx + vy + vw*5)*2;
  wheel_rpm[2] = ( vx - vy + vw*5)*2;
  wheel_rpm[3] = (-vx - vy + vw*5)*2;


  for (uint8_t i = 0; i < 4; i++)
    {
      if (abs(wheel_rpm[i]) > max)
        max = abs(wheel_rpm[i]);
    }

  if (max > MAX_WHEEL_RPM)
    {
      float rate = MAX_WHEEL_RPM / max;
      for (uint8_t i = 0; i < 4; i++)
        wheel_rpm[i] *= rate;
    }
  memcpy(speed, wheel_rpm, 4*sizeof(int16_t));//内存拷贝函数，rpm一分钟旋转量
}
/**
  * @brief  nitialize chassis motor pid parameter
  * @usage  before chassis loop use this function
  */
void chassis_param_init(void)//底盘参数初始化
{
  memset(&chassis, 0, sizeof(chassis_t));
  chassis.ctrl_mode      = CHASSIS_SEPARATE;
  chassis.last_ctrl_mode = CHASSIS_RELAX;

  chassis.position_ref = 0;
//  chassis_rotate_flag = 0;

  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,24,0.3, 10); //24 0.3 10    38.0f,3.0f, 40
    }
//  PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 70, 5.0f, 0.01,30.0f);//xisanhao
}
