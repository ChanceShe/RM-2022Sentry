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
		if(key_scan1(GPIOA,KEY_GPIO_PIN1) == KEY_S3)
		{
			RED_LED_TOGGLE();
		}
	}
}

