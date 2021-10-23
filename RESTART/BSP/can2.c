#include "main.h"

void CAN2_Init(void) //CAN2初始化
{
	GPIO_InitTypeDef  GPIO_In;
	CAN_InitTypeDef CAN_In;
	CAN_FilterInitTypeDef CAN_FilterIn;
	NVIC_InitTypeDef  NVIC_In;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);                											 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN2时钟	
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 
	
	GPIO_In.GPIO_Pin   = GPIO_Pin_12| GPIO_Pin_13;
  GPIO_In.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_In.GPIO_OType = GPIO_OType_PP;
  GPIO_In.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_In.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_In);	
	
	NVIC_In.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_In.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_In.NVIC_IRQChannelSubPriority = 1;
  NVIC_In.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_In);
    
  NVIC_In.NVIC_IRQChannel = CAN2_TX_IRQn;
  NVIC_In.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_In.NVIC_IRQChannelSubPriority = 1;
  NVIC_In.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_In);	
	
  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_In);
	CAN_In.CAN_Prescaler= 3;
	CAN_In.CAN_Mode = CAN_Mode_Normal;
	CAN_In.CAN_SJW  = CAN_SJW_1tq;
	CAN_In.CAN_BS1  = CAN_BS1_9tq;
	CAN_In.CAN_BS2  = CAN_BS2_4tq;
	CAN_In.CAN_TTCM = DISABLE;//非时间触发通信模式
	CAN_In.CAN_ABOM = ENABLE;//软件自动离线管理	
	CAN_In.CAN_AWUM = DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_In.CAN_NART = DISABLE;//禁止报文自动传送 
	CAN_In.CAN_RFLM = DISABLE;//报文不锁定,新的覆盖旧的  
	CAN_In.CAN_TXFP = DISABLE;//优先级由报文标识符决定 
	CAN_Init(CAN2, &CAN_In);
	
	CAN_FilterIn.CAN_FilterIdHigh = 0x0000;
	CAN_FilterIn.CAN_FilterIdLow  = 0x0000;
	CAN_FilterIn.CAN_FilterMaskIdHigh= 0x0000;
	CAN_FilterIn.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterIn.CAN_FilterFIFOAssignment= 0;
	CAN_FilterIn.CAN_FilterNumber = 14;
	CAN_FilterIn.CAN_FilterMode   = CAN_FilterMode_IdMask;
	CAN_FilterIn.CAN_FilterScale  = CAN_FilterScale_32bit;
	CAN_FilterIn.CAN_FilterActivation= ENABLE;
	CAN_FilterInit(&CAN_FilterIn);
	
  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
  CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
		
}

void CAN2_TX_IRQHandler(void) //CAN2发送中断函数
{

  if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
}


void CAN2_RX0_IRQHandler(void) //CAN2接收中断函数
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FF0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message);  
        //电机编码器数据处理
        Can2ReceiveMsgProcess(&rx_message);
    }
}
