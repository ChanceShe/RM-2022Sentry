#ifndef __MAIN_H__
#define __MAIN_H__
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_spi.h"
#include "string.h"
#include "stdarg.h"
#include "config.h"
#include "math.h"

#include "delay.h"
#include "iwdg.h"
#include "common.h"
#include "pid.h"
#include "ramp.h"
#include "Fifo.h"
#include "LostCounter.h"
#include "init.h"
#include "bsp_flash.h"
#include "Protocol.h"
#include "Judge.h"

#include "timer.h"
#include "pwm.h"
#include "LED.h"
#include "key_scan.h"

#include "usart1.h"
#include "usart3.h"
#include "output2vs.h"
#include "usart4.h"

#include "hi220.h"


#include "can1.h"
#include "can2.h"
#include "CanbusTask.h"

#include "ControlTask.h"
#include "CanbusTask.h"

#include "RemoteTask.h"
#include "GimbalTask.h"
#include "ShootTask.h"
#include "AutoshootTask.h"
#include "ModeswitchTask.h"
#include "SuperviseTask.h"
#include "IOTask.h"

#include "protobuf-c.h"
#include "Attack.pb-c.h"
#include "Signal.pb-c.h"
#include "protocol.pb-c.h"
#include "filter.h"
#include "kalman_filter.h"

extern int testnum1;		
extern int testnum2;
extern int testnum3;
extern int testnum4;

#endif 
