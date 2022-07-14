#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__
#include "main.h"

#define CHASSIS_SPEED_ATTENUATION   (1.0f)
#define MAX_WHEEL_RPM  7400		//�����ת��

//����������⴫����
#define SENSOR_TYPE 1		//0:����,1:�����״�

//����ģʽ���򷽰�
#define	CRAZY_DIR_CHANGE_MODE	1		//0:�������(�����Ӿ�Ԥ��),1:ȫ����˶�(���Ʋ������ִ�)

//�������Ʒ���
#define POWER_LIMIT_MODE   1   //0:�ϸ�����,1:ʹ�û�������

//ɲ��ʹ��
#define BRAKE_EN			1

//����������������
#define WARNING_ENERGY 	80

//���������ʼ��� p=i*v*I_TIMES_V_TO_WATT;i��ֱ�ӷ���������� v�ǵ��ת��
#define  I_TIMES_V_TO_WATT    0.0000231f    //I -16384~+16384 V .filter_rate
//������ȼ��� p=i^2*FACTOR_2+i*FACTOR_1+FACTOR0; i��ֱ�ӷ����������-16384~16384 ʹ������ʾ������ֵ��matlab���
#define FACTOR_2	0.000000161f
#define FACTOR_1	-0.0000229f
#define FACTOR_0  0.458f

#define POWER_CONTROL_DEFAULT \
{	0,\
	0,\
	0,\
	0,\
}\


typedef enum
{
  CHASSIS_RELAX          = 0,
  CHASSIS_STOP           = 1,
  CHASSIS_REMOTE  			 = 2,		//ң�������Ƶ���
  CHASSIS_PATROL		 		 = 3,		//Ѳ��ģʽ
} chassis_mode_e;

typedef enum
{
    switch_off 	    = 0,
    switch_on       = 1,
} opt_switch_e;			//		����״̬

typedef enum
{
    position_middle         = 0,
    position_left           = 1,
    position_right          = 2,
} position_e;    			//   �Զ�ģʽ�µ�λ��״̬

typedef enum
{
	direction_left						= 0,
	direction_right						= 1,
	direction_stop						= 2,
	direction_stopleft				= 3,
	direction_stopright				= 4,
}	direction_e;					//�Զ�ģʽ�µ��˶�����

typedef struct
{
	uint8_t 				crazyflag;
	uint16_t 			 	crazytime;
	uint8_t 			 	crazychangetime;
	uint8_t		 			crazyspeeddir;
	int 			 			crazyspeed;
}	crazydata_t;					//����ģʽ����

typedef enum
{
  limit_strict         	= 0,
  limit_buffer          = 1,
	limit_current					= 2,
} powerlimit_e;    			//��������ģʽ


typedef struct
{
  double           vx; // forward/back
  
  chassis_mode_e  ctrl_mode;
  chassis_mode_e  last_ctrl_mode;
	
  int16_t         wheel_speed_fdb;
  int16_t         wheel_speed_ref;
  int16_t         current;
  powerlimit_e		powerlimit;
	
	opt_switch_e		opt_switch_l,opt_switch_r;

	crazydata_t			crazydata;
	
	position_e 			position;
	direction_e 		direction;

} chassis_t;

extern chassis_t chassis;


extern uint8_t brake_en;
extern uint32_t chassis_patrol_time;

void chassis_param_init(void);//���̲�����ʼ��
void chassis_task(void);
static void chassis_remote_handle(void);
static void chassis_patrol_handle(void);
static void chassis_stop_handle(void);


typedef struct
{
    int32_t Cnt_Power_Judge_Recieved;
    int32_t Time_10ms;
    int32_t Cnt_Power_Judge_Recieved_Pre;
    uint8_t Flag_Judge_Control;
    float 	K_Output;
} Power_Control_Struct;
extern Power_Control_Struct Power_Control;
extern float power_limit_rate;
extern int32_t total_cur_limit;
extern int32_t total_cur;
void power_limit_handle ( void );



#endif
