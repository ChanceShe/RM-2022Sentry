#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__
#include "main.h"

#define CHASSIS_SPEED_ATTENUATION   (1.0f)
#define MAX_WHEEL_RPM  7400		//轮最大转速

//左右立柱检测传感器
#define SENSOR_TYPE 1		//0:光电管,1:激光雷达

//暴走模式变向方案
#define	CRAZY_DIR_CHANGE_MODE	1		//0:击打变向(反制视觉预测),1:全随机运动(反制操作手手打)

//功率限制方案
#define POWER_LIMIT_MODE   1   //0:严格限制,1:使用缓冲能量

//刹车使能
#define BRAKE_EN			1

//缓冲能量警告下限
#define WARNING_ENERGY 	80

//电机输出功率计算 p=i*v*I_TIMES_V_TO_WATT;i是直接发给电调的数 v是电机转速
#define  I_TIMES_V_TO_WATT    0.0000231f    //I -16384~+16384 V .filter_rate
//电机发热计算 p=i^2*FACTOR_2+i*FACTOR_1+FACTOR0; i是直接发给电调的数-16384~16384 使用虚拟示波器读值后matlab拟合
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
  CHASSIS_REMOTE  			 = 2,		//遥控器控制底盘
  CHASSIS_PATROL		 		 = 3,		//巡逻模式
} chassis_mode_e;

typedef enum
{
    switch_off 	    = 0,
    switch_on       = 1,
} opt_switch_e;			//		光电管状态

typedef enum
{
    position_middle         = 0,
    position_left           = 1,
    position_right          = 2,
} position_e;    			//   自动模式下的位置状态

typedef enum
{
	direction_left						= 0,
	direction_right						= 1,
	direction_stop						= 2,
	direction_stopleft				= 3,
	direction_stopright				= 4,
}	direction_e;					//自动模式下的运动方向

typedef struct
{
	uint8_t 				crazyflag;
	uint16_t 			 	crazytime;
	uint8_t 			 	crazychangetime;
	uint8_t		 			crazyspeeddir;
	int 			 			crazyspeed;
}	crazydata_t;					//暴走模式参数

typedef enum
{
  limit_strict         	= 0,
  limit_buffer          = 1,
	limit_current					= 2,
} powerlimit_e;    			//功率限制模式


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

void chassis_param_init(void);//底盘参数初始化
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
