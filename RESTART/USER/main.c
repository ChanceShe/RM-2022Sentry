#include "main.h"

u8 SYS_START = 0;
int testnum1 = 0;		
int testnum2 = 0;
int testnum3 = 0;
int testnum4 = 0;

int main(void)
{
	BSP_Init();							//�����ʼ��
	ControtLoopTaskInit();	//���������ʼ��
	SYS_START = 1;
	RED_LED_OFF();
  while(1)
	{
    GREEN_LED_ON();
//		LASER_ON();		
	}
}

