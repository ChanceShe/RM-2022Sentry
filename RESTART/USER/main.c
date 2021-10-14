#include "main.h"

int main(void)
{
	BSP_Init();	
  while(1)
	{
    GREEN_LED_ON();
		delay_ms(500);
		GREEN_LED_OFF();
		delay_ms(500);
	}
}

