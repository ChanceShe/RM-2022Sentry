2021.10.28
�����˶�����
0:��ǰ 1:��ǰ 2:��� 3:�Һ�
wheel_rpm[0] = (robot_vy - robot_vx + robot_vw * K) * (1.0);
wheel_rpm[1] = (robot_vy + robot_vx - robot_vw * K) * (1.0);
wheel_rpm[2] = (robot_vy - robot_vx - robot_vw * K) * (1.0);
wheel_rpm[3] = (robot_vy + robot_vx + robot_vw * K) * (1.0);

2021.11.15
