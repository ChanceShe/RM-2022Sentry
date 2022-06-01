#ifndef _REOMTE_TASK_H_
#define _REOMTE_TASK_H_
#include "main.h"
#include "ramp_second.h"

#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u
#define REMOTE_CONTROLLER_STICK_OFFSET      1024u   //ң���������м�λ��ֵ
#define STICK_TO_CHASSIS_SPEED_REF_FACT     0.6f
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.001f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.0015f//0.005f

#define REMOTE_SWITCH_VALUE_UP         		0x01u
#define REMOTE_SWITCH_VALUE_DOWN				0x02u
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

//#define PITCH_MAX -1.0f
//#define PITCH_MIN -30.0f
//#define Init_Pitch_Angle -15.0f

#define PITCH_MAX -45.0f
#define PITCH_MIN -80.0f
#define Init_Pitch_Angle -55.0f
#define YAW_MAX 80				//��̨�Ƕȵķ�Χ
#define YAW_MIN -80

//ң����
typedef __packed struct
{
	int16_t ch0;		//��ҡ��X
	int16_t ch1;		//��ҡ��Y
	int16_t ch2;		//��ҡ��X
	int16_t ch3;		//��ҡ��Y
	int8_t s1;			//�󲦸�
	int8_t s2;			//�Ҳ���
}Remote;

//���
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

//����
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


//����ģʽ:ң����/�������/ֹͣ����
typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP = 2,
}InputMode_e;

//���˶���ö��
typedef enum
{
    FROM1TO2,
    FROM1TO3,
    FROM2TO1,
    FROM3TO1,
    FROM3TO2,
} RC_SWITCH_ACTION_e;

//Ħ����״̬ö��
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

//remote data process����ǰ��/����/��ת�ٶȸ���
typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

//remote data process��̨pitch/yaw�ٶȸ���
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

extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern Gimbal_Ref_t GimbalRef;


void RemoteTaskInit(void);								//б�³�ʼ��
extern RampGen_t frictionRamp;  					//Ħ����б��
#define FRICTION_RAMP_TICK_COUNT			100 //��תб��б��
#define FRICTION_RAMP_OFF_TICK_COUNT	30	//ͣתб��б��
extern FrictionWheelState_e friction_wheel_state ;


void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val);		//ң������ֵ
void RemoteControlProcess(refrom_mainboard_t *rc);
void RemoteDataPrcess(refrom_mainboard_t *rc);		//ң�������ݴ�����CAN1�ж���
uint8_t IsRemoteBeingAction ( void );
InputMode_e GetInputMode( void );
Shoot_State_e GetShootState ( void );
void SetShootState ( Shoot_State_e v );
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val);		//ң���󲦸�ģʽ���
void Remote_Rotate_Reverse_Control(RemoteSwitch_t *sw, uint8_t val);		//ң���󲦸�ģʽС����
void SetInputMode(refrom_mainboard_t *rc);					//����ģʽ����

#endif
