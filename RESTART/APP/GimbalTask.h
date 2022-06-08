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
    GIMBAL_AUTO_MODE   = 5,
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
    AUTO_PATROL         = 0,	//未识别到目标自动巡逻
    AUTO_FOLLOW          = 1,	//识别到目标自动射击
} auto_mode_e;    //   自动模式下的状态

typedef struct
{
    float Yaw_Angle_Pre;
    float Yaw_Angle_Now;
    float Pit_Angle_Pre;
    float Pit_Angle_Now;
    uint32_t Time_Sample;
    float Angular_Yaw_Speed;
    float Angular_Yaw_Speed_Pre;
    float Angular_Pit_Speed;
    float Angular_Pit_Speed_Pre;
    float Yaw_Acceleration;
    float Pit_Acceleration;
    uint32_t time1;
    uint32_t time2;
    float time_error;
    float yaw_angle_error;
    float pit_angle_error;
    float time_delay;
    float Army_Speed_Yaw;
    float Army_Speed_Pit;
		int Continuity_timer;
} Speed_Prediction_t;
typedef struct
{
//    Speed_Prediction_t Speed_Prediction;
//    Speed_Prediction_t Speed_Prediction_Kalman;
//    float Filtered_Angular_Yaw_Speed;
//    float Filtered_Angular_Pit_Speed;
//    float Filtered_Yaw_Acceleration;
//    float Filtered_Pit_Acceleration;
    uint8_t	  Recognized_Flag;
    uint16_t   Recognized_Timer;
    int16_t   Continue_Recognized_Cnt;
//    float Err_Pixels_Yaw;
//    float Err_Pixels_Pit;
//    float Distance;
//    float Ballistic_Compensation;
//    float Horizontal_Compensation;
    float Yaw_Gimbal_Delay_Compensation;
    float Pit_Gimbal_Delay_Compensation;
//    uint8_t Image_Gimbal_Delay_Compensation_Flag;
//    float Delta_Dect_Angle_Pit;
//    float Delta_Dect_Angle_Yaw;
//    uint16_t   Continue_Large_Err_Cnt;
//    double shoot_pitch_angle;
//    float Armor_yaw;
//    float Armor_pit;
		float target_pit;
		float target_yaw;
} Gimbal_Auto_Shoot_t;

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
		
		float pit_lost_feb;
		float yaw_lost_feb;	
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

} gimbal_t;

extern gimbal_t gim;
extern float Init_Yaw_Angle;
extern RampGen_t GMPitchRamp ;
extern RampGen_t GMYawRamp ;
extern auto_mode_e auto_mode;

static void cascade_pid_ctrl ( void );

void gimbal_task( void );
static void gimbal_init_handle	( void );
static void no_action_handle	( void );
static void gimbal_remote_handle( void );
static void gimbal_auto_handle( void );
static void gimbal_patrol_handle( void );
static void gimbal_follow_handle( void );		//运动跟随模式
static void auto_shoot_task( void );
void gimbal_param_init ( void );

int Speed_Continuity_Timmer(float speed);

#endif
