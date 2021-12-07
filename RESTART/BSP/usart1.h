#ifndef __USART1_H__
#define __USART1_H__
#include "main.h"

/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/

#define  BSP_USART1_DMA_RX_BUF_LEN               64u                   
 
#define BSP_USART1_RX_BUF_SIZE_IN_FRAMES         (BSP_USART1_RX_BUF_SIZE / RC_FRAME_LENGTH)
#define  RC_FRAME_LENGTH                            18u

/*
*********************************************************************************************************
*                                             FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void USART1_IRQHandler(void);
//static void USART1_FIFO_Init(void);
void *USART1_GetRxBuf(void);
void USART1_Configuration(uint32_t baud_rate);
void RemoteDataPrcess(uint8_t *pData);

#endif
