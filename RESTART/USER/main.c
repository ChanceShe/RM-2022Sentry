#include "main.h"

float testnum1 = 0;
float testnum2 = 0;
float testnum3 = 0;
float testnum4 = 0;

int main(void)
{
	BSP_Init();							//�����ʼ��
	ControtLoopTaskInit();	//���������ʼ��
  while(1)
	{
    GREEN_LED_TOGGLE();
	}
}
