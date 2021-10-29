#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__
#include "main.h"
void motorcontrol_init(void);
void moter_control(void); //�����������
extern volatile Encoder CM11Encoder;
extern volatile Encoder CM21Encoder;
#define CAN1_MOTOR1_ID  0x201
#define CAN2_MOTOR1_ID  0x201

#endif
