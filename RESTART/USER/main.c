#include "main.h"


int testflag = 0;		
int main(void)
{
	BSP_Init();							//外设初始化
	ControtLoopTaskInit();	//控制任务初始化
  while(1)
	{
    GREEN_LED_TOGGLE();
		if(key_scan(GPIOB,KEY_GPIO_PIN) == KEY_S3)
		{
			RED_LED_TOGGLE();
		}
		
	}
}

