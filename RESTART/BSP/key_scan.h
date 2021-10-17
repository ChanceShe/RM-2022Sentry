#ifndef __KEY_SCAN_H__
#define __KEY_SCAN_H__

#include "main.h"

#define SCAN_TIME 10 //
#define DOWN_TIME 10//
#define HOLD_TIME (long unsigned int)4000//




///*下面的方式是通过直接操作库函数方式读取IO*/
//#define KEY0    GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)    //PA5
//#define KEY1    GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)    //PA6


typedef enum key_states_e
{
    KEY_DOWN,
    KEY_UP

} key_states_e;

typedef enum
{
    KEY_S1,
    KEY_S2,
    KEY_S3,
    KEY_S4
} key_msg_e;



#define   KEY_PORT        	    GPIOB
#define   KEY_GPIO_PIN		      GPIO_Pin_14







key_states_e key_read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
key_msg_e key_scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
key_msg_e key_scan1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


void KEY_Init(void);	//IO初始化


#endif

