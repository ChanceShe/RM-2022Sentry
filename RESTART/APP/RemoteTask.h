#ifndef _REOMTE_TASK_H_
#define _REOMTE_TASK_H_
#include "main.h"
#include "ramp_second.h"

#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u
#define REMOTE_CONTROLLER_STICK_OFFSET      1024u   //遥控器拨杆中间位置值
#define STICK_TO_CHASSIS_SPEED_REF_FACT     1.6f
#define STICK_TO_PITCH_ANGLE_INC_FACT       2.0f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.05f//0.005f

#define REMOTE_SWITCH_VALUE_UP         		0x01u
#define REMOTE_SWITCH_VALUE_DOWN			0x02u
#define REMOTE_SWITCH_VALUE_CENTRAL			0x03u

#define REMOTE_SWITCH_CHANGE_1TO3      (uint8_t)((REMOTE_SWITCH_VALUE_UP << 2) | REMOTE_SWITCH_VALUE_CENTRAL)
#define REMOTE_SWITCH_CHANGE_2TO3      (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 2) | REMOTE_SWITCH_VALUE_CENTRAL)
#define REMOTE_SWITCH_CHANGE_3TO1      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_UP)
#define REMOTE_SWITCH_CHANGE_3TO2      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_DOWN)

#define REMOTE_SWITCH_CHANGE_1TO3TO2   (uint8_t)((REMOTE_SWITCH_VALUE_UP << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_DOWN))

#define REMOTE_SWITCH_CHANGE_2TO3TO1   (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_UP))

#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u


//遥控器
typedef __packed struct
{
	int16_t ch0;		//右摇杆X
	int16_t ch1;		//右摇杆Y
	int16_t ch2;		//左摇杆X
	int16_t ch3;		//左摇杆Y
	int8_t s1;			//左拨杆
	int8_t s2;			//右拨杆
}Remote;

//鼠标
typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;

//按键
typedef	__packed struct
{
	uint16_t v;
	uint16_t last_v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;


//输入模式:遥控器/键盘鼠标/停止运行
typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	RELAX = 2,
}InputMode_e;

//拨杆动作枚举
typedef enum
{
    FROM1TO2,
    FROM1TO3,
    FROM2TO1,
    FROM3TO1,
    FROM3TO2,
} RC_SWITCH_ACTION_e;

//摩擦轮状态枚举
typedef enum
{
    FRICTION_WHEEL_OFF = 0,
    FRICTION_WHEEL_START_TURNNING = 1,
    FRICTION_WHEEL_ON = 2,
    FRICTION_WHEEL_STOP_TURNNING = 3,
} FrictionWheelState_e;

typedef enum
{
    NOSHOOTING = 0,
    SHOOTING = 1,
} Shoot_State_e;
typedef enum
{
    NORMAL_SHOOTING = 0,//正常射击模式，鼠标按住一直转
    ONE_SHOOTING = 1,//鼠标单击一次只发一颗弹丸
    THREE_SHOOTING = 2,//单击一次三连发
} Shooting_State_e; //弹丸射击模式

//remote data process底盘前后/左右/旋转速度给定
typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

//remote data process云台pitch/yaw速度给定
typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
}Gimbal_Ref_t;


//to detect the action of the switch
typedef struct RemoteSwitch_t
{
	 uint8_t switch_value_raw;            // the current switch value
	 uint8_t switch_value1;				  //  last value << 2 | value
	 uint8_t switch_value2;				  //
	 uint8_t switch_long_value; 		  //keep still if no switching
	 uint8_t switch_value_buf[REMOTE_SWITCH_VALUE_BUF_DEEP]; 
	 uint8_t buf_index;
	 uint8_t buf_last_index;
	 uint8_t buf_end_index;
}RemoteSwitch_t;

extern RC_Ctl_t RC_CtrlData;
extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern Gimbal_Ref_t GimbalRef;


void RemoteTaskInit(void);								//斜坡初始化
extern RampGen_t frictionRamp;  					//摩擦轮斜坡
#define FRICTION_RAMP_TICK_COUNT			100 //起转斜坡斜率
#define FRICTION_RAMP_OFF_TICK_COUNT	30	//停转斜坡斜率


void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val);		//遥控器读值
void RemoteControlProcess(Remote *rc);		//遥控器控制模式处理
void RemoteDataPrcess(uint8_t *pData);		//遥控器数据处理
void SetInputMode(Remote *rc);
InputMode_e GetInputMode();
Shoot_State_e GetShootState ( void );
Shooting_State_e GetShootingState ( void );
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val);		//遥控左拨杆模式射击
void Remote_Rotate_Reverse_Control(RemoteSwitch_t *sw, uint8_t val);		//遥控左拨杆模式小陀螺

#endif
