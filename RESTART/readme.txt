2021.10.28
麦轮运动解算
0:右前 1:左前 2:左后 3:右后
wheel_rpm[0] = (robot_vy - robot_vx + robot_vw * K) * (1.0);
wheel_rpm[1] = (robot_vy + robot_vx - robot_vw * K) * (1.0);
wheel_rpm[2] = (robot_vy - robot_vx - robot_vw * K) * (1.0);
wheel_rpm[3] = (robot_vy + robot_vx + robot_vw * K) * (1.0);

2021.11.15
