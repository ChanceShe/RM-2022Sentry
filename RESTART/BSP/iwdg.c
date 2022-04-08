#include "main.h"
void IWDG_Configuration(void)	
{
	//���Ź���ʱ��32KHZ  ���ʱ��t = ( (4*2^Prescaler)*Reload )/32  ms
	//   Prescaler = 4     Reload = 625   ���ʱ��t=1000ms
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //IWDG->PR IWDG->RLR  
	IWDG_SetPrescaler(IWDG_Prescaler_4); 
	IWDG_SetReload(10);   //4ms   2*Reload
	IWDG_ReloadCounter(); 
	IWDG_Enable(); 
}	
