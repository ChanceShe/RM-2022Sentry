#include "main.h"

int testnum1 = 0;
int testnum2 = 0;
int testnum3 = 0;
int testnum4 = 0;

int main(void)
{
	BSP_Init();							//�����ʼ��
	ControtLoopTaskInit();	//���������ʼ��
  while(1)
	{
    GREEN_LED_TOGGLE();
	}
}
