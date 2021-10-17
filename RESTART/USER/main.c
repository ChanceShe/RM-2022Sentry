#include "main.h"



int main(void)
{
	BSP_Init();	
	IWDG_Configuration();
  while(1)
	{
    GREEN_LED_TOGGLE();
		if(key_scan(GPIOB,KEY_GPIO_PIN) == KEY_S3)
		{
			RED_LED_TOGGLE();
		}
		
	}
}

