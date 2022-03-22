#include "main.h"

uint32_t time_tick_1ms = 0;
int dir=1;
uint8_t   is_judge_on = 0;

void Control_Task(void)
{
	
  time_tick_1ms++;
	IWDG_ReloadCounter();
	if(time_tick_1ms%2 == 0)
  {
		modeswitch_task();	
		gimbal_task();
	}		

//	if(time_tick_1ms%5 == 0)
//  {
//		shot_task();		
//	}		

	
	OutData[0] = 0;
	OutData[1] = 0;
	OutData[2] = 20000;
	OutData[3] = 30000;
	OutPut_Data(OutData);		
}
	
