#include "main.h"

u8 SYS_START = 0;
int testflag = 0;
int testnum = 0;
int main(void)
{
	BSP_Init();							//外设初始化
	ControtLoopTaskInit();	//控制任务初始化
	SYS_START = 1;
	RED_LED_OFF();
  while(1)
	{
    GREEN_LED_ON();
//		LASER_ON();		
	}
}

