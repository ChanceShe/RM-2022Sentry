#ifndef __MAIN_H__
#define __MAIN_H__
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_spi.h"
#include "string.h"
#include <stdarg.h>

#include "delay.h"
#include "iwdg.h"
#include "common.h"
#include "bsp.h"
#include "pid.h"

#include "timer.h"
#include "pwm.h"
#include "LED.h"
#include "key_scan.h"

#include "usart1.h"
#include "usart3.h"
#include "output2vs.h"
#include "usart4.h"

#include "can1.h"
#include "can2.h"
#include "CanbusTask.h"
#include "MotorControl.h"

#include "ControlTask.h"
#include "CanbusTask.h"

#include "RemoteTask.h"
#include "ChassisTask.h"
#include "SuperviseTask.h"

#include "ramp.h"
#include "fifo.h"
#include "LostCounter.h"


#endif 
