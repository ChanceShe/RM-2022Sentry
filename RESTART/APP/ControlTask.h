#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_
#include "main.h"

typedef enum
{
  red = 0,
  blue = 1,
  unkown = 2,
} robot_color_e;
extern robot_color_e robot_color ;
extern uint8_t ShootFlag;

extern uint32_t time_tick_1ms;
extern uint8_t   is_judge_on;

void Control_Task(void);

#endif

