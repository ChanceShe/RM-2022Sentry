#include "main.h"

u8 SYS_START = 0;
float testnum1 = 0;		
float testnum2 = 0;
float testnum3 = 0;
float testnum4 = 0;

int main(void)
{



	ControtLoopTaskInit();	//���������ʼ��
	BSP_Init();							//�����ʼ��
	SYS_START = 1;
	RED_LED_OFF();
  while(1)
	{
    GREEN_LED_ON();
		LASER_ON();
	}
}
