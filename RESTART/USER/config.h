#ifndef __SYS_H__
#define __SYS_H__

//==========================================================
// <o> REMOTE_SHOOT  - ң�����󲦸˵Ĺ���ѡ��
// <0=> С����
// <1=> ����
#define REMOTE_SHOOT        0                 //1:ң�����󲦸�Ϊ����  0:С����

#define GMPitchEncoder_Offset  	4557
#define GMYawEncoder_Offset   	7000
#define GYRO_REAL_X_OFFSET 			0.00559780467
#define GYRO_REAL_Y_OFFSET 			-0.0156064359
#define GYRO_REAL_Z_OFFSET 			-0.0091483118

//#define ARMY_SPEED_PREDICTION   1
//#define ENABLE_KALMAN_FILTER    1

////==========================================================
////����ͷ��ǹ�����ĵİ�װƫ���-Yaw����
//#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		  -1.0f  //-6.0
////����ͷ��ǹ�����ĵİ�װƫ���-Pitch����
//#define ANGLE_BETWEEN_GUN_CAMERA 					1.0f  //-3.4  
////����ͷ��ǹ�����ĵĴ�ֱ����
//#define HEIGHT_BETWEEN_GUN_CAMERA 				44.2f
////ת�������ͷ����ľ���
//#define DISTANCEC_BETWEEN_GUN_CAMERA      130.79f
////�������
//#define FOCAL_LENGTH                      6.15f
////���泤
//#define TARGET_SURFACE_LENGTH             6.4e-3f
////�����
//#define TARGET_SURFACE_WIDTH              4.8e-3f
////���سߴ�mm
//#define IMAGE_LENGTH                      4.8e-3f
////Զ����ʱǹ�ܽǶȲ���
//#define ANGLE_COMPENSATION_LONG_DISTANCE 	1.6f
////ͼ�����̨�����ӳ�ʱ�� - �� /
//#define YAW_IMAGE_GIMBAL_DELAY_TIME				25e-3f//s
//#define PIT_IMAGE_GIMBAL_DELAY_TIME				20e-3f//s

#endif
