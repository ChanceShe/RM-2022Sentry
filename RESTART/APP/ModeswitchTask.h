#ifndef __MODE_SW_TASK_H__
#define __MODE_SW_TASK_H__

#include "main.h"

void modeswitch_task (void);
void get_gimbal_mode ( void );  // ������������һ�����ģʽ����̨
void gimbal_mode_handle ( void );
static void get_last_mode ( void );

#endif
