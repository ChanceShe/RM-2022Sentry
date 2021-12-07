#ifndef _INIT_H_
#define _INIT_H_

/*
*********************************************************************************************************
*                                           PERIPH BUF SIZES 
*********************************************************************************************************
*/
#define     BSP_USART1_RX_BUF_SIZE            128u
#define     BSP_USART3_RX_BUF_SIZE            1024u
#define     BSP_USART3_TX_BUF_SIZE            512u
#define 		PREPARE_TIME_TICK_MS						  1000      //prapare time in ms

void BSP_Init(void);
void ControtLoopTaskInit(void);

#endif

