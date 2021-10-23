#ifndef __PWM_H__
#define __PWM_H__



#define PWM1  TIM3->CCR1
#define PWM2  TIM3->CCR2

#define OpenDoor  PWM1=135;

void PWM_Configuration(void);
 
#endif /* __PWM_H__*/

