#ifndef __MAIN_H__
#define __MAIN_H__
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_spi.h"
#include "string.h"
#include "stdarg.h"
#include "config.h"

#include "delay.h"
#include "iwdg.h"
#include "common.h"
#include "pid.h"
#include "ramp.h"
#include "Fifo.h"
#include "LostCounter.h"
#include "init.h"
#include "bsp_flash.h"

#include "timer.h"
#include "pwm.h"
#include "LED.h"
#include "key_scan.h"

#include "usart1.h"
#include "usart3.h"
#include "output2vs.h"
#include "usart4.h"

#include "hi220.h"
#include "protocal.h"


#include "can1.h"
#include "can2.h"
#include "CanbusTask.h"

#include "ControlTask.h"
#include "CanbusTask.h"

#include "RemoteTask.h"
#include "ChassisTask.h"
#include "GimbalTask.h"
#include "ShootTask.h"
#include "ModeswitchTask.h"
#include "SuperviseTask.h"
#include "IOTask.h"

#endif 
