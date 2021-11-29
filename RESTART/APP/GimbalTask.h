#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__
#include "main.h"

typedef enum
{
    GIMBAL_RELAX         = 0,
    GIMBAL_INIT          = 1,
    GIMBAL_NO_ARTI_INPUT = 2,
    GIMBAL_REMOTE_MODE   = 3,
    GIMBAL_TRACK_ARMOR   = 4,
    GIMBAL_PATROL_MODE   = 5,
    GIMBAL_POSITION_MODE = 6,
	  GIMBAL_TEST_MODE     = 7,
} gimbal_mode_e;

typedef enum
{
    NO_ACTION = 0,
    IS_ACTION,
} action_mode_e;

typedef enum
{
    CMD_NO = 0,
    CMD_CALI_FIVE,
    CMD_CALI_NINE,
    CMD_TARGET_NUM
} gimbal_cmd_e;

typedef struct
{
    /* position loop */
    float yaw_angle_ref;
    float pit_angle_ref;
    float yaw_angle_fdb;
    float pit_angle_fdb;
    /* speed loop */
    float yaw_speed_ref;
    float pit_speed_ref;
    float yaw_speed_fdb;
    float pit_speed_fdb;
} gim_pid_t;

typedef struct
{
    /* unit: degree */
    float pit_relative_angle;
    float yaw_relative_angle;
    float gyro_angle;
    /* uint: degree/s */
    float yaw_palstance;
    float pit_palstance;
} gim_sensor_t;

typedef struct
{
    action_mode_e ac_mode;
    float         action_angle;
    uint8_t       no_action_flag;
    uint32_t      no_action_time;
} no_action_t;

typedef struct
{
    /* ctrl mode */
    gimbal_mode_e ctrl_mode;
    gimbal_mode_e last_ctrl_mode;

    /* gimbal information */
    gim_sensor_t  sensor;
    float         ecd_offset_angle;
    float         yaw_offset_angle;

    /* gimbal ctrl parameter */
    gim_pid_t     pid;
    no_action_t   input;

    /* read from flash */
    int32_t       pit_center_offset;
    int32_t       yaw_center_offset;

    gimbal_cmd_e  auto_ctrl_cmd;
} gimbal_t;

extern float Init_Yaw_Angle  , Init_Pitch_Angle;

static void cascade_pid_ctrl ( void );

void gimbal_task( void );
static void gimbal_init_handle	( void );
static void gimbal_stop_handle	( void );
static void gimbal_remote_handle( void );
static void gimbal_patrol_handle( void );
void gimbal_param_init ( void );

#endif
