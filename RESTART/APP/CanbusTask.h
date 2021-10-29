#ifndef __CANBUS_TASK_H
#define __CANBUS_TASK_H
#include "main.h"

/* CAN Bus 1 */  


/* CAN Bus 2 */  
#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID           0x201//右前 逆时针
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID           0x204


#define RATE_BUF_SIZE 6						//滤波数量
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

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;

void Can2ReceiveMsgProcess(CanRxMsg * msg);
void Can1ReceiveMsgProcess(CanRxMsg * msg);
void CAN2_Send_Msg(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void CAN2_Send_Msg1(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq);
void CAN1_Send_Msg(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void CAN1_Send_Msg1(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq);


#endif


