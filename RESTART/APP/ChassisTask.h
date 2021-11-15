#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__
#include "main.h"

#define CHASSIS_SPEED_ATTENUATION   (1.0f)
#define MAX_WHEEL_RPM  7400		//�����ת��
#define MAX_CHASSIS_VR_SPEED 170		//���������ת���ٶ�


typedef enum
{
  CHASSIS_RELAX          = 0,
  CHASSIS_STOP           = 1,
  MANUAL_SEPARATE_GIMBAL = 2,		//ң�������Ƶ�����
  MANUAL_FOLLOW_GIMBAL   = 3,		//ң�������Ƶ�����̨
  DODGE_MODE             = 4,
  AUTO_SEPARATE_GIMBAL   = 5,
  AUTO_FOLLOW_GIMBAL     = 6,
  CHASSIS_PATROL		 		 = 7,		//Ѳ��ģʽ
} chassis_mode_e;

typedef struct
{
  double           vx; // forward/back
  double           vy; // left/right
  double           vw; // 
  
  chassis_mode_e  ctrl_mode;
  chassis_mode_e  last_ctrl_mode;

  float           gyro_angle;
  float           gyro_palstance;

  int16_t         wheel_speed_fdb;
  int16_t         wheel_speed_ref;
  int16_t         current;
  
  int32_t         position_ref;
  uint8_t         follow_gimbal;
} chassis_t;


extern chassis_t chassis;

void chassis_param_init(void);//���̲�����ʼ��

void chassis_task(void);
void follow_gimbal_handle(void);
static void chassis_stop_handle(void);



#endif
