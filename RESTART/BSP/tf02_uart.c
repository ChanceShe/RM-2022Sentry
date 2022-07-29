#include "main.h"

/*-----USART2_TX-----PA2-----*/
/*-----USART2_RX-----PA3-----*/
/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/


uint8_t USART2_DMA_RX_BUF[USART2_RX_BUF_LENGTH];
uint8_t USART2_DMA_TX_BUF[USART2_TX_BUF_LENGTH];
uint8_t USART3_DMA_RX_BUF[USART3_RX_BUF_LENGTH];
uint8_t USART3_DMA_TX_BUF[USART3_TX_BUF_LENGTH];

TF02_Data_t TF02_L,TF02_R;


void TF02_USART_Config(void)
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
    DMA_InitTypeDef dma;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3,ENABLE);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2|GPIO_PinSource3,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10|GPIO_PinSource11,GPIO_AF_USART3);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
		//USART2
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA,&gpio);

    usart.USART_BaudRate = 115200;          // speed 10byte/ms
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2,&usart);

		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
		
		DMA_DeInit(DMA1_Stream5);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr		= (uint32_t)(&USART2->DR);
    dma.DMA_Memory0BaseAddr   		= (uint32_t)&USART2_DMA_RX_BUF;
    dma.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize				= USART2_RX_BUF_LENGTH;//sizeof(USART1_DMA_RX_BUF);
    dma.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc 				= DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
    dma.DMA_Mode 					= DMA_Mode_Normal;
    dma.DMA_Priority 				= DMA_Priority_Medium;
    dma.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold 			= DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, &dma);
    DMA_Cmd(DMA1_Stream5, ENABLE);
		
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority =0;
    nvic.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&nvic);		
	
    USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
    USART_Cmd(USART2,ENABLE);
		
		//USART3
		gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&gpio);

    usart.USART_BaudRate = 115200;          // speed 10byte/ms
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3,&usart);

		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
		
		DMA_DeInit(DMA1_Stream1);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr		= (uint32_t)(&USART3->DR);
    dma.DMA_Memory0BaseAddr   		= (uint32_t)&USART3_DMA_RX_BUF;
    dma.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize				= USART3_RX_BUF_LENGTH;//sizeof(USART1_DMA_RX_BUF);
    dma.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc 				= DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
    dma.DMA_Mode 					= DMA_Mode_Normal;
    dma.DMA_Priority 				= DMA_Priority_Medium;
    dma.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold 			= DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1, &dma);
    DMA_Cmd(DMA1_Stream1, ENABLE);
		
    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority =1;
    nvic.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&nvic);		
	
    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
    USART_Cmd(USART3,ENABLE);
		memset(&TF02_L,0,sizeof(TF02_Data_t));
		memset(&TF02_R,0,sizeof(TF02_Data_t));
}
void USART2_IRQHandler(void)
{  
		if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)    //接收中断
		{
			(void)USART2->SR;
		  (void)USART2->DR;
			DMA_Cmd(DMA1_Stream5, DISABLE);  
			DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
			TF02DataDeal (&TF02_R, USART2_DMA_RX_BUF );  //自瞄接收函数
			DMA_SetCurrDataCounter(DMA1_Stream5,USART2_RX_BUF_LENGTH);
			DMA_Cmd(DMA1_Stream5, ENABLE);
		}       
}
void USART3_IRQHandler(void)
{  
		if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)    //接收中断
		{
			(void)USART3->SR;
		  (void)USART3->DR;
			DMA_Cmd(DMA1_Stream1, DISABLE);  
			DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
			TF02DataDeal (&TF02_L, USART3_DMA_RX_BUF );  //自瞄接收函数
			DMA_SetCurrDataCounter(DMA1_Stream1,USART3_RX_BUF_LENGTH);
			DMA_Cmd(DMA1_Stream1, ENABLE);
		}       
}

void TF02DataDeal(TF02_Data_t *TF02_Data,u8 *buf)
{
	TF02_Data->Dist = buf[2]|(buf[3]<<8);
	TF02_Data->Strength = buf[4]|(buf[5]<<8);
	TF02_Data->RecieveFlag = 1;
}
