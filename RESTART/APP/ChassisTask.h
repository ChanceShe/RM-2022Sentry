#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__
#include "main.h"

#define CHASSIS_SPEED_ATTENUATION   (1.0f)
#define MAX_WHEEL_RPM  7400		//轮最大转速
#define MAX_CHASSIS_VR_SPEED 170		//底盘最大旋转角速度


typedef enum
{
  CHASSIS_RELAX          = 0,
  CHASSIS_STOP           = 1,
  MANUAL_SEPARATE_GIMBAL = 2,
  MANUAL_FOLLOW_GIMBAL   = 3,
  DODGE_MODE             = 4,
  AUTO_SEPARATE_GIMBAL   = 5,
  AUTO_FOLLOW_GIMBAL     = 6,
  CHASSIS_ROTATE         = 7,
  CHASSIS_REVERSE        = 8,
  CHASSIS_CHANGE_REVERSE = 9,
  CHASSIS_SEPARATE 		 = 10,
  CHASSIS_AUTO_SUP       = 11,
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

  int16_t         wheel_speed_fdb[4];
  int16_t         wheel_speed_ref[4];
  int16_t         current[4];
  
  int32_t         position_ref;
  uint8_t         follow_gimbal;
} chassis_t;


extern chassis_t chassis;


void chassis_task(void);
void chassis_separate_handle(void);
void chassis_remove_task(void);
static void chassis_stop_handle(void);

static void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);


#endif
