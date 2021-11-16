#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__
#include "main.h"
#define PID_SHOOT_MOTOR_SPEED           -550
#define PID_SHOOT_NOJUDGE_MOTOR_SPEED   -550
#define FRICTION_SPEED                  920
\
typedef enum
{
    SHOT_DISABLE       = 0,
    REMOTE_CTRL_SHOT   = 1,
    KEYBOARD_CTRL_SHOT = 2,
    SEMIAUTO_CTRL_SHOT = 3,
    AUTO_CTRL_SHOT     = 4,
} shoot_mode_e;

typedef __packed struct
{
    /* shoot task relevant param */
    shoot_mode_e ctrl_mode;
    uint8_t      shoot_cmd;
    uint32_t     c_shoot_time;   //continuous
    uint8_t      c_shoot_cmd;
    uint8_t      fric_wheel_run; //run or not
    uint16_t     fric_wheel_spd;
    uint16_t     ref_shot_bullets;
    uint16_t     shot_bullets;
    uint16_t     remain_bullets;
    float        total_speed;
    float        limit_heart0;
    uint16_t     max_heart0;
    uint16_t     handle_timescouter;
    uint16_t     cooling_ratio;
} shoot_t;

void shot_task ( void );
void shoot_friction_handle ( void );					//Ä¦²ÁÂÖ
static void shoot_bullet_handle ( void );			//²¦ÅÌ
void shot_param_init ( void );								//³õÊ¼»¯

extern uint8_t friction_rotor;
#endif
