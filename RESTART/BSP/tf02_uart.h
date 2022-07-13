#ifndef __TF02_UART_H
#define __TF02_UART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"

#define USART2_RX_BUF_LENGTH 16
#define USART2_TX_BUF_LENGTH 16
#define USART3_RX_BUF_LENGTH 16
#define USART3_TX_BUF_LENGTH 16

typedef struct
{
	uint16_t Dist;
	uint16_t Strength;
	uint8_t RecieveFlag;
} TF02_Data_t;

extern TF02_Data_t TF02_L,TF02_R;

void TF02_USART_Config(void);
void TF02DataDeal(TF02_Data_t *TF02_Data,u8 *buf);

#endif
