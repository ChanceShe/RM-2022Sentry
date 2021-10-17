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
}
	
	