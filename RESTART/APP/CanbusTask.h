#ifndef __CANBUS_TASK_H
#define __CANBUS_TASK_H
#include "main.h"

/* CAN Bus 1 */  

#define CAN_BUS1_FRICTION_MOTOR1_FEEDBACK_MSG_ID       0x201    		//   云台摩擦轮  
#define CAN_BUS1_FRICTION_MOTOR2_FEEDBACK_MSG_ID       0x202    		//   云台摩擦轮   
#define CAN_BUS1_FRICTION_MOTOR3_FEEDBACK_MSG_ID       0x203    		//   云台摩擦轮  
#define CAN_BUS1_FRICTION_MOTOR4_FEEDBACK_MSG_ID       0x204    		//   云台摩擦轮   

/* CAN Bus 2 */ 
#define CAN_BUS2_YAW_MOTOR_FEEDBACK_MSG_ID             0x206    		// 	 云台yaw
#define CAN_BUS2_PITCH_MOTOR_FEEDBACK_MSG_ID           0x205   			//   云台pitch
#define CAN_BUS2_POKE1_FEEDBACK_MSG_ID            		 0x201    		//拨盘
#define CAN_BUS2_POKE2_FEEDBACK_MSG_ID            		 0x202    		//拨盘
#define CAN_BUS2_SLAVE_FEEDBACK_MSG_ID   		      		 0x408   			//上云台向下云台传输数据ID

#define RATE_BUF_SIZE 6						//滤波数量

typedef struct
{
    uint16_t ch2;
    uint16_t ch3;
    uint8_t s1;
    uint8_t s2;
    uint8_t color;
    uint8_t VehicleShootFlag;
    uint8_t JudgeShootFlag;
    uint16_t shoot_heart0;
} refrom_mainboard_t;  //   云台通信接收结构体
extern refrom_mainboard_t refromData;

typedef struct{
	int32_t raw_value;   				    //编码器不经处理的原始值
	int32_t last_raw_value;				  //上一次的编码器原始值
	int32_t ecd_value;              //经过处理后连续的编码器值
	int32_t diff;						        //两次编码器之间的差值
	int32_t temp_count;             //计数用
	uint8_t buf_count;					    //滤波更新buf用
	int32_t ecd_bias;					      //初始编码器值	
	int32_t ecd_raw_rate;				    //通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];//buf，for filter
	int32_t round_cnt;			        //圈数
	int32_t filter_rate;				    //速度
	float ecd_angle;					      //角度
}Encoder;

typedef struct
{
    uint8_t s1;
    uint8_t s2;
    int16_t ch2;
    int16_t ch3;
	  uint8_t Communication_Flag;  //下云台发送通讯标志位
    uint8_t location_Flag;
	
} refrom_info_t;  //   云台通信接收结构体

//CAN1
extern volatile Encoder Friction1Encoder ;		//云台摩擦轮
extern volatile Encoder Friction2Encoder ;		//云台摩擦轮
extern volatile Encoder Friction3Encoder ;		//云台摩擦轮
extern volatile Encoder Friction4Encoder ;		//云台摩擦轮

//CAN2
extern volatile Encoder GMYawEncoder ;				//云台Yaw
extern volatile Encoder GMPitchEncoder ;			//云台Pitch
extern volatile Encoder Poke1Encoder;					//拨盘
extern volatile Encoder Poke2Encoder;					//拨盘

void Can2ReceiveMsgProcess(CanRxMsg * msg);
void Can1ReceiveMsgProcess(CanRxMsg * msg);
void CAN2_Send_Msg(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void CAN2_Send_Msg1(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq);
void CAN1_Send_Msg(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void CAN1_Send_Msg1(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq);
void CAN2_Gimbal_Msg ( int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq );

extern refrom_mainboard_t refromData;
void revice_main_information ( refrom_mainboard_t *data, CanRxMsg * msg );
static void mainBoard_control ( refrom_mainboard_t *rcInfomation );
static void set_imput_mode ( refrom_mainboard_t *rc );

#endif


