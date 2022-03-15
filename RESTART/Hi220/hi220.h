#ifndef __HI220_H
#define __HI220_H
#include "STM32F4xx.h"

#define LENGTH_USER_ID_0x90 			1
#define LENGTH_ACC_0xa0 					6
#define LENGTH_LINEAR_ACC_0xa5 		6
#define LENGTH_ANG_VEL_0xb0 			6
#define LENGTH_MAG_0xc0 					6
#define LENGTH_EULER_ANG_s16_0xd0 6
#define LENGTH_EULER_ANG_f_0xd9 	12
#define LENGTH_QUATERNION_0xd1 		16
#define LENGTH_AIR_PRESS_0xf0 		4
#define USART6_RX_BUF_LENGTH 100

typedef struct {
					uint8_t User_ID;
					int16_t Acc_X;
					int16_t Acc_Y;
					int16_t Acc_Z;
					int16_t Linear_Acc_X;
					int16_t Linear_Acc_Y;
					int16_t Linear_Acc_Z;
					int16_t Ang_Velocity_X;
					int16_t Ang_Velocity_Y;
					int16_t Ang_Velocity_Z;
					int16_t Mag_X;
					int16_t Mag_Y;
					int16_t Mag_Z;
					int16_t Euler_Angle_Pitch_s16;
					int16_t Euler_Angle_Roll_s16;
					int16_t Euler_Angle_Yaw_s16;
					
					union
					{
						float Euler_Angle_Pitch_f;
						uint8_t Euler_Angle_Pitch_u8[4];
					}Euler_Angle_Pitch;
					
					union
					{
						float Euler_Angle_Roll_f;
						uint8_t Euler_Angle_Roll_u8[4];
					}Euler_Angle_Roll;

					union
					{
						float Euler_Angle_Yaw_f;
						uint8_t Euler_Angle_Yaw_u8[4];
					}Euler_Angle_Yaw;
					
					union
					{
						float Quaternion_W_f;
						uint8_t Quaternion_W_u8[4];
					}Quaternion_W;		

					union
					{
						float Quaternion_X_f;
						uint8_t Quaternion_X_u8[4];
					}Quaternion_X;	
						union
					{
						float Quaternion_Y_f;
						uint8_t Quaternion_Y_u8[4];
					}Quaternion_Y;	
						union
					{
						float Quaternion_Z_f;
						uint8_t Quaternion_Z_u8[4];
					}Quaternion_Z;	
					float Euler_Angle_Pitch_s16_2_f;
					float Euler_Angle_Roll_s16_2_f;
					float Euler_Angle_Yaw_s16_2_f;	
} HI220_Stucture;
typedef struct
{
	union
	{
		uint8_t Flag_Configured;
		struct
		{
			uint8_t Eout_Configured 	: 1;
			uint8_t ODR_Configured 	: 1;
			uint8_t BAUD_Configured 	: 1;
			uint8_t SETPTL_Configured: 1;
			uint8_t MODE_Configured 	: 1;
			uint8_t MCAL_Configured 	: 1;
			uint8_t Reserve 					: 2;
			
		}Bits;
	}Hi220_Flag_Configured;
	union
	{
		uint8_t Flag_Reconfig;
		struct
		{
			uint8_t Eout_Reconfig 	: 1;
			uint8_t ODR_Reconfig 	: 1;
			uint8_t BAUD_Reconfig 	: 1;
			uint8_t SETPTL_Reconfig: 1;
			uint8_t MODE_Reconfig 	: 1;
			uint8_t MCAL_Reconfig 	: 1;
			uint8_t Reserve 				: 2;
			
		}Bits;
	}Hi220_Flag_Reconfig;
}Hi220_Flags_t;

extern HI220_Stucture HI220_Data_From_Usart;
extern float yaw_Speed,pitch_Speed;

extern float yaw_Angle,pitch_Angle,roll_Angle; 
extern float pitch_Gyro,yaw_Gyro;

void Hi220_Init(void);
void Hi220_Reset(void);
void USART6_Configuration_For_Hi220(void);
void Hi220_getYawPitchRoll(void);
#endif


