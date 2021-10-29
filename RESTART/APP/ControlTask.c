#include "main.h"

uint32_t time_tick_1ms = 0;
int dir=1;

void Control_Task(void)
{
	
  time_tick_1ms++;
	IWDG_ReloadCounter();
	if(time_tick_1ms % 10 == 0)
	{
		if(PWM1>=200 || PWM1<=100)
		{
			dir=!dir;
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
	
	if(time_tick_1ms%10 == 0)
  {
			chassis_task();
	}		

	
	OutData[0] = CM11Encoder.filter_rate;
	OutData[1] = pid_motor1.set;
	OutData[2] = 20000;
	OutData[3] = 30000;
	OutPut_Data(OutData);		
}
	
	