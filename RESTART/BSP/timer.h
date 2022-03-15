#ifndef __TIMER_H__
#define __TIMER_H__
#include "main.h"

uint32_t Get_Time_Micros(void);
extern u8 SYS_START;

void TIM6_Configuration(void);
void TIM4_Configuration(void);
void TIM2_Configuration(void);
void TIM8_Configuration(void);

#endif
