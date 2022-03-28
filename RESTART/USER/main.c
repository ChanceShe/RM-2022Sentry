#include "main.h"

float testnum1 = 0;
float testnum2 = 0;
float testnum3 = 0;
float testnum4 = 0;

int main(void)
{
	BSP_Init();							//外设初始化
	ControtLoopTaskInit();	//控制任务初始化
  while(1)
	{
    GREEN_LED_TOGGLE();
	}
}
