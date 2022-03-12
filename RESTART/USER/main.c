#include "main.h"

int main(void)
{
	BSP_Init();							//外设初始化
	ControtLoopTaskInit();	//控制任务初始化
  while(1)
	{
    GREEN_LED_TOGGLE();
	}
}
