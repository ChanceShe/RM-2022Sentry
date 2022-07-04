#ifndef __CH100_DMA_H
#define __CH100_DMA_H
#include "main.h"

extern float pitch_Angle, yaw_Angle, roll_Angle; 
extern float pitch_Gyro, yaw_Gyro, roll_Gyro;
extern float x_Acc, y_Acc, z_Acc;

#define CH100_RX_BUFF_SIZE 100

__packed typedef struct
{
uint8_t tag; /* ????:0x91 */
uint8_t id; /* ??ID */
uint8_t rev[2];
float prs; /* ?? */
uint32_t ts; /* ??? */
float acc[3]; /* ??? */
float gyr[3]; /* ??? */
float mag[3]; /* ?? */
float eul[3]; /* ???:
Roll,Pitch,Yaw */
float quat[4]; /* ??? */
}id0x91_t;

 /* Definition for USART_CH100 resources ******************************************/
  #define USART_CH100                           USART6
  #define USART_CH100_CLK                       RCC_APB2Periph_USART6
  #define USART_CH100_CLK_INIT                  RCC_APB2PeriphClockCmd
  #define USART_CH100_IRQn                      USART6_IRQn
  #define USART_CH100_IRQHandler                USART6_IRQHandler

  #define USART_CH100_TX_PIN                    GPIO_Pin_6
  #define USART_CH100_TX_GPIO_PORT              GPIOC
  #define USART_CH100_TX_GPIO_CLK               RCC_AHB1Periph_GPIOC
  #define USART_CH100_TX_SOURCE                 GPIO_PinSource6
  #define USART_CH100_TX_AF                     GPIO_AF_USART6

  #define USART_CH100_RX_PIN                    GPIO_Pin_7
  #define USART_CH100_RX_GPIO_PORT              GPIOC                   
  #define USART_CH100_RX_GPIO_CLK               RCC_AHB1Periph_GPIOC
  #define USART_CH100_RX_SOURCE                 GPIO_PinSource7
  #define USART_CH100_RX_AF                     GPIO_AF_USART6

  /* Definition for DMAx resources ********************************************/
  #define USART_CH100_DR_ADDRESS                ((uint32_t)USART6 + 0x04) 

  #define USART_CH100_DMA                       DMA2
  #define USART_CH100_DMAx_CLK                  RCC_AHB1Periph_DMA2
     
  #define USART_CH100_TX_DMA_CHANNEL            DMA_Channel_5
  #define USART_CH100_TX_DMA_STREAM             DMA2_Stream7
  #define USART_CH100_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF7
  #define USART_CH100_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF7
  #define USART_CH100_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF7
  #define USART_CH100_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF7
  #define USART_CH100_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF7
              
  #define USART_CH100_RX_DMA_CHANNEL            DMA_Channel_5
  #define USART_CH100_RX_DMA_STREAM             DMA2_Stream1
  #define USART_CH100_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
  #define USART_CH100_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
  #define USART_CH100_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
  #define USART_CH100_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
  #define USART_CH100_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1

//  #define USART_CH100_DMA_TX_IRQn               DMA2_Stream6_IRQn
//  #define USART_CH100_DMA_RX_IRQn               DMA2_Stream1_IRQn
//  #define USART_CH100_DMA_TX_IRQHandler         DMA2_Stream6_IRQHandler
//  #define USART_CH100_DMA_RX_IRQHandler         DMA2_Stream1_IRQHandler

extern void ch100_USART_Config(void);
#endif