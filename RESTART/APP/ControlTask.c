#include "main.h"

uint32_t time_tick_1ms = 0;
extern int dir=0;

void Control_Task(void)
{
	
  time_tick_1ms++;
	IWDG_ReloadCounter();
	if(time_tick_1ms % 10 == 0)
	{
		if(PWM1>=200)
		{
			dir=0;
		}
		if(PWM1<=100)
		{
			dir=1;
		}
		if(dir)
		{
			PWM1++;
		}
		else 
		{
			PWM1--;
		}
	}
	OutData[0] = time_tick_1ms%40000;
	OutData[1] = 10000;
	OutData[2] = 20000;
	OutData[3] = 30000;
		OutPut_Data(OutData);		
}
	
	