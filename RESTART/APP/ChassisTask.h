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
  CHASSIS_CONTROL   = 3,		//ң�������Ƶ�����̨
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
  
  uint8_t         follow_gimbal;
} chassis_t;


extern chassis_t chassis;

typedef enum
{
    sensor_off 	    = 0,
    sensor_on       = 1,
} sensor_state_e;			//		����״̬

typedef enum
{
    position_middle         = 0,
    position_left           = 1,
    position_right          = 2,
} position_e;    			//   �Զ�ģʽ�µ�λ��״̬
extern position_e robot_position;
typedef enum
{
	direction_left						= 0,
	direction_right						= 1,
	direction_stop						= 2,
	direction_stopleft				= 3,
	direction_stopright				= 4,
}	direction_e;					//�Զ�ģʽ�µ��˶�����
extern direction_e robot_direction;

void chassis_param_init(void);//���̲�����ʼ��

void chassis_task(void);
void chassis_control_handle(void);
void chassis_patrol_handle(void);
static void chassis_stop_handle(void);



#endif
