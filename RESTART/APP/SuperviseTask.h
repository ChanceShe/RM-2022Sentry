#ifndef _DEBUG_H_
#define _DEBUG_H_
#include "stm32f4xx.h"


#define LOST_COUNTER_NUM                             11u

#define LOST_COUNTER_INDEX_MOTOR1                    3u    //green:red:green 1 1 1 
#define LOST_COUNTER_INDEX_MOTOR2                    4u    //green:red:green 1 2 1 
#define LOST_COUNTER_INDEX_MOTOR3                    5u    //green:red:green 1 3 1 
#define LOST_COUNTER_INDEX_MOTOR4                    6u    //green:red:green 1 4 1 
#define LOST_COUNTER_INDEX_MOTOR5                    7u    //green:red:green 1 5 1 
#define LOST_COUNTER_INDEX_MOTOR6                    8u    //green:red:green 1 6 1 
#define LOST_COUNTER_INDEX_DEADLOCK                  9u    //red:red:red 1:1:1

uint32_t *GetLostCounter(uint8_t index);

#endif
