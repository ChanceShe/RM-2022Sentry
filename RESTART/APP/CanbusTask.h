#ifndef __CANBUS_TASK_H
#define __CANBUS_TASK_H
#include "main.h"

/* CAN Bus 1 */  

#define CAN_BUS1_FRICTION_MOTOR1_FEEDBACK_MSG_ID       0x201    		//   ��̨Ħ����  
#define CAN_BUS1_FRICTION_MOTOR2_FEEDBACK_MSG_ID       0x202    		//   ��̨Ħ����   
#define CAN_BUS1_FRICTION_MOTOR3_FEEDBACK_MSG_ID       0x203    		//   ��̨Ħ����  
#define CAN_BUS1_FRICTION_MOTOR4_FEEDBACK_MSG_ID       0x204    		//   ��̨Ħ����   

/* CAN Bus 2 */ 
#define CAN_BUS2_YAW_MOTOR_FEEDBACK_MSG_ID             0x206    		// 	 ��̨yaw
#define CAN_BUS2_PITCH_MOTOR_FEEDBACK_MSG_ID           0x205   			//   ��̨pitch
#define CAN_BUS2_POKE1_FEEDBACK_MSG_ID            		 0x201    		//����
#define CAN_BUS2_POKE2_FEEDBACK_MSG_ID            		 0x202    		//����
#define CAN_BUS2_SLAVE_FEEDBACK_MSG_ID   		      		 0x408   			//����̨������̨��������ID
#define CAN_BUS2_JUDGE_FEEDBACK_MSG_ID   		      		 0x406   			//����������̨�������ϵͳ����ID

#define RATE_BUF_SIZE 6						//�˲�����

typedef struct
{
    uint16_t ch2;
    uint16_t ch3;
    uint8_t s1;
    uint8_t s2;
    uint8_t color;
    uint8_t VehicleShootFlag;
    uint8_t JudgeShootFlag;
    uint8_t shoot_heart_l;
		uint8_t shoot_heart_r;
} refrom_mainboard_t;  //   ��̨ͨ�Ž��սṹ��
extern refrom_mainboard_t refromData;
extern uint8_t currentid;

typedef struct
{
    uint16_t robot1_HP;
    uint16_t robot3_HP;
    uint16_t robot4_HP;
    uint16_t robot5_HP;
} judge_mainboard_t;  //   ��̨ͨ�Ž��սṹ��
extern judge_mainboard_t judgeData;

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



//CAN1
extern volatile Encoder Friction1Encoder ;		//��̨Ħ����
extern volatile Encoder Friction2Encoder ;		//��̨Ħ����
extern volatile Encoder Friction3Encoder ;		//��̨Ħ����
extern volatile Encoder Friction4Encoder ;		//��̨Ħ����

//CAN2
extern volatile Encoder GMYawEncoder ;				//��̨Yaw
extern volatile Encoder GMPitchEncoder ;			//��̨Pitch
extern volatile Encoder Poke1Encoder;					//����
extern volatile Encoder Poke2Encoder;					//����

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


