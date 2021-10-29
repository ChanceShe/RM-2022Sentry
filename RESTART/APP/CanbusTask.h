#ifndef __CANBUS_TASK_H
#define __CANBUS_TASK_H
#include "main.h"

/* CAN Bus 1 */  


/* CAN Bus 2 */  
#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID           0x201//��ǰ ��ʱ��
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID           0x204


#define RATE_BUF_SIZE 6						//�˲�����
typedef struct{
	int32_t raw_value;   				    //���������������ԭʼֵ
	int32_t last_raw_value;				  //��һ�εı�����ԭʼֵ
	int32_t ecd_value;              //��������������ı�����ֵ
	int32_t diff;						        //���α�����֮��Ĳ�ֵ
	int32_t temp_count;             //������
	uint8_t buf_count;					    //�˲�����buf��
	int32_t ecd_bias;					      //��ʼ������ֵ	
	int32_t ecd_raw_rate;				    //ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];//buf��for filter
	int32_t round_cnt;			        //Ȧ��
	int32_t filter_rate;				    //�ٶ�
	float ecd_angle;					      //�Ƕ�
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


