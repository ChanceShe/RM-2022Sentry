#ifndef __SYS_H__
#define __SYS_H__

//==========================================================
// <o> REMOTE_SHOOT  - 遥控器左拨杆的功能选择
// <0=> 小陀螺
// <1=> 发射
#define REMOTE_SHOOT        0                 //1:遥控器左拨杆为发射  0:小陀螺

#define GMPitchEncoder_Offset  	4557
#define GMYawEncoder_Offset   	7000
#define GYRO_REAL_X_OFFSET 			0.00559780467
#define GYRO_REAL_Y_OFFSET 			-0.0156064359
#define GYRO_REAL_Z_OFFSET 			-0.0091483118

//#define ARMY_SPEED_PREDICTION   1
//#define ENABLE_KALMAN_FILTER    1

////==========================================================
////摄像头和枪管中心的安装偏差角-Yaw方向
//#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		  -1.0f  //-6.0
////摄像头和枪管中心的安装偏差角-Pitch方向
//#define ANGLE_BETWEEN_GUN_CAMERA 					1.0f  //-3.4  
////摄像头和枪管中心的垂直距离
//#define HEIGHT_BETWEEN_GUN_CAMERA 				44.2f
////转轴和摄像头焦点的距离
//#define DISTANCEC_BETWEEN_GUN_CAMERA      130.79f
////相机焦距
//#define FOCAL_LENGTH                      6.15f
////靶面长
//#define TARGET_SURFACE_LENGTH             6.4e-3f
////靶面宽
//#define TARGET_SURFACE_WIDTH              4.8e-3f
////像素尺寸mm
//#define IMAGE_LENGTH                      4.8e-3f
////远距离时枪管角度补偿
//#define ANGLE_COMPENSATION_LONG_DISTANCE 	1.6f
////图像和云台控制延迟时间 - 秒 /
//#define YAW_IMAGE_GIMBAL_DELAY_TIME				25e-3f//s
//#define PIT_IMAGE_GIMBAL_DELAY_TIME				20e-3f//s

#endif
