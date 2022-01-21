2021.10.28
麦轮运动解算
0:右前 1:左前 2:左后 3:右后
wheel_rpm[0] = (robot_vy - robot_vx + robot_vw * K) * (1.0);
wheel_rpm[1] = (robot_vy + robot_vx - robot_vw * K) * (1.0);
wheel_rpm[2] = (robot_vy - robot_vx - robot_vw * K) * (1.0);
wheel_rpm[3] = (robot_vy + robot_vx + robot_vw * K) * (1.0);

2021.11.15
底盘模式切换：
云台初始化时：底盘停止CHASSIS_STOP
其余状态时：输入遥控模式REMOTE_INPUT：		底盘遥控器控制CHASSIS_REMOTE
		   输入键鼠控制KEY_MOUSE_INPUT：		底盘巡逻CHASSIS_PATROL
云台模式切换：

2022.01.19
protocal.h改为protocol.h
bsp.c改为init.c