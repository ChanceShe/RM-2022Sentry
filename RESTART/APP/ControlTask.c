#include "main.h"

uint32_t time_tick_1ms = 0;
int dir=1;
//****裁判系统相关标志****
robot_color_e robot_color ;
char* command = "sc_r";
uint8_t   is_judge_on = 0;
uint8_t   JudgeShoot = 0;
//***********************
void Control_Task(void)
{
	
  time_tick_1ms++;
	IWDG_ReloadCounter();
	

	if(time_tick_1ms%5 == 0)
  {
		upperboard_send_to_mainboard ( CAN2, RC_CtrlData.rc.ch2, RC_CtrlData.rc.ch3, RC_CtrlData.rc.s1, RC_CtrlData.rc.s2,\
															robot_color, judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat,\
															 VehicleShootFlagLow, JudgeShoot );
		modeswitch_task();	
	}	
	
	if(time_tick_1ms%10 == 1)
  {
			chassis_task();
	}		

	
	OutData[0] = 0;
	OutData[1] = 0;
	OutData[2] = 20000;
	OutData[3] = 30000;
	OutPut_Data(OutData);		
}
	
