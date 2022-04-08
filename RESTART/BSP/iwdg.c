#include "main.h"
void IWDG_Configuration(void)	
{
	//看门狗的时钟32KHZ  溢出时间t = ( (4*2^Prescaler)*Reload )/32  ms
	//   Prescaler = 4     Reload = 625   溢出时间t=1000ms
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //IWDG->PR IWDG->RLR  
	IWDG_SetPrescaler(IWDG_Prescaler_4); 
	IWDG_SetReload(10);   //4ms   2*Reload
	IWDG_ReloadCounter(); 
	IWDG_Enable(); 
}	
