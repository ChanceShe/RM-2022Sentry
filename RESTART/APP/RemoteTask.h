#ifndef _REOMTE_TASK_H_
#define _REOMTE_TASK_H_
#include "main.h"
#include "ramp_second.h"

#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u
#define REMOTE_CONTROLLER_STICK_OFFSET      1024u   //ң���������м�λ��ֵ
#define STICK_TO_CHASSIS_SPEED_REF_FACT     0.6f
#define STICK_TO_PITCH_ANGLE_INC_FACT       2.0f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.05f//0.005f


typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;
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


//remote data process����ǰ�������ٶȸ���
typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

//remote data process��̨pitch\yaw�ٶȸ���
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


void RemoteTaskInit(void);		//б�³�ʼ��
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val);		//ң������ֵ
void RemoteControlProcess(Remote *rc);		//ң��������ģʽ����
void RemoteDataPrcess(uint8_t *pData);		//ң�������ݴ���
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val);		//ң���󲦸�ģʽ���
void Remote_Rotate_Reverse_Control(RemoteSwitch_t *sw, uint8_t val);		//ң���󲦸�ģʽС����

#endif
