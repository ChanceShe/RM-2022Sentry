#include "main.h"

int main(void)
{
	BSP_Init();							//�����ʼ��
	ControtLoopTaskInit();	//���������ʼ��
  while(1)
	{
    GREEN_LED_TOGGLE();
	}
}
