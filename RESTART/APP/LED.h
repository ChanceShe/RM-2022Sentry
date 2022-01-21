#ifndef __LED_H__
#define __LED_H__

void Led_Configuration(void);
void Laser_Configuration ( void );

#define GREEN_LED_ON()      GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define GREEN_LED_OFF()     GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define GREEN_LED_TOGGLE()      GPIO_ToggleBits(GPIOC, GPIO_Pin_1)

#define RED_LED_ON()            GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define RED_LED_OFF()           GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define RED_LED_TOGGLE()        GPIO_ToggleBits(GPIOC, GPIO_Pin_2)
	
#define BOTH_LED_TOGGLE()\
GPIO_ToggleBits(GPIOC, GPIO_Pin_1);\
GPIO_ToggleBits(GPIOC, GPIO_Pin_2)

#define LASER_ON()  GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LASER_OFF()  GPIO_ResetBits(GPIOA, GPIO_Pin_8)

#endif
