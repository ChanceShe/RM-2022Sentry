#ifndef __SYS_H_、_
#define __SYS_H__

//==========================================================
// <o> REMOTE_SHOOT  - 遥控器左拨杆的功能选择
// <0=> 小陀螺
// <1=> 发射
#define REMOTE_SHOOT        0                 //1:遥控器左拨杆为发射  0:小陀螺

#define GMPitchEncoder_Offset  	4444
#define GMYawEncoder_Offset   	6435
#define GYRO_REAL_X_OFFSET 			0.00559780467
#define GYRO_REAL_Y_OFFSET 			-0.0156064359
#define GYRO_REAL_Z_OFFSET 			-0.0091483118

//==========================================================
#define Share_remotecontrols  0      //是否共用遥控器      共用时别插接收机，否则会产生遥控器的串口中断

#endif
