#include "main.h"


int testflag = 0;		
int main(void)
{
	BSP_Init();							//�����ʼ��
	ControtLoopTaskInit();	//���������ʼ��
  while(1)
	{
    GREEN_LED_TOGGLE();
//		LASER_ON();		
	}
}

