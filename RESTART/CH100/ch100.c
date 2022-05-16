/************************************************************
 *  @file ch100.c
 *  @version 1.0
 *  @date April 2022
 *	@author SheChengqi
 *  @brief ch100 contrl realization(CH100陀螺仪控制)
 *
 *  @copyright 2022 UPC_RPS. All rights reserved.
 *
 ************************************************************/
#include "main.h"

float pitch_Angle, yaw_Angle, roll_Angle; 	//三轴角度
float pitch_Gyro, yaw_Gyro, roll_Gyro;			//三轴角加速度
float x_Acc, y_Acc, z_Acc;									//三向加速度
uint32_t imu_time_1ms = 0;									//陀螺仪内置时钟

/* common type conversion */
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
#define I2(p) (*((int16_t  *)(p)))
static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}
static float    R4(uint8_t *p) {float    r; memcpy(&r,p,4); return r;}

uint8_t decode_succ;          	 //陀螺仪数据接收标志，成功返回1，不成功返回0
raw_t ch100 = {0};               // IMU stram read/control struct

void CH100_getDATA() 
{  
	volatile static float Pitch_temp,Roll_temp, Yaw_temp;
	volatile static float Last_pitch_temp, Last_yaw_temp, Last_roll_temp;
	volatile static int Pitch_count, Roll_count, Yaw_count;


	pitch_Gyro = -ch100.imu.gyr[0] * 0.1f;
	roll_Gyro = ch100.imu.gyr[1] * 0.1f;
  yaw_Gyro = ch100.imu.gyr[2] * 0.1f;
	
	
	/*	get pitch angle	*/
	pitch_Angle = -ch100.imu.eul[0]-90;
	
	/*	get roll angle	*/
	roll_Angle = ch100.imu.eul[1];
	
	/*	get yaw angle	*/
	Last_yaw_temp = Yaw_temp;
	Yaw_temp = ch100.imu.eul[2];
	if(Yaw_temp-Last_yaw_temp>=324)  
	{
		Yaw_count--;
	}
	else if (Yaw_temp-Last_yaw_temp<=-324)
	{
		Yaw_count++;
	}
	yaw_Angle = -(Yaw_temp + Yaw_count*360); 


	/*	get x acceleration	*/
	x_Acc = ch100.imu.acc[0];
	/*	get y acceleration	*/
	y_Acc = ch100.imu.acc[0];
	/*	get z acceleration	*/	
	z_Acc = ch100.imu.acc[0];
	
	imu_time_1ms = ch100.imu.timestamp;
}

/* CH100 init */
void USART6_Configuration_For_CH100( void )
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;
	DMA_InitTypeDef dma;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);


	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6复用为USART6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOC7复用为USART6

	//USART1端口配置
	gpio.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9与GPIOA10
	gpio.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	gpio.GPIO_PuPd 	= GPIO_PuPd_NOPULL; //上拉
	GPIO_Init(GPIOC,&gpio); //初始化PC6，PC7

	USART_DeInit(USART6);
	usart.USART_BaudRate = 115200;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &usart);
	
//	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
//    
//	DMA_DeInit(DMA2_Stream1);
//	dma.DMA_Channel = DMA_Channel_5;
//	dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART6->DR);
//	dma.DMA_Memory0BaseAddr   	= (uint32_t)&ch100.buf[0];
//	dma.DMA_DIR 			    = DMA_DIR_PeripheralToMemory;
//	dma.DMA_BufferSize			= MAXBUFLEN;
//	dma.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
//	dma.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
//	dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
//	dma.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
//	dma.DMA_Mode 				= DMA_Mode_Normal;
//	dma.DMA_Priority 			= DMA_Priority_Medium;
//	dma.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
//	dma.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
//	dma.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
//	dma.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
//	DMA_Init(DMA2_Stream1, &dma);
//	DMA_Cmd(DMA2_Stream1, ENABLE);
		
	nvic.NVIC_IRQChannel = USART6_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 3;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 3;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);        //USART RX RENE INTERRUPT  ENABLED
	USART_Cmd(USART6,ENABLE); 
}

void USART6_IRQHandler(void)
{
	uint8_t ch;
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(USART6);	
		}
    /* 合并单字节并解码 */
    decode_succ = ch_serial_input(&ch100, ch);	//成功返回1，不成功返回0代表陀螺仪数据丢失
}

static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currect_crc = crc;
}

/* parse the payload of a frame and feed into data section */
static int parse_data(raw_t *raw)
{
    int ofs = 0, i = 0;
    uint8_t *p = &raw->buf[CH_HDR_SIZE];
    memset(raw->item_code, 0, sizeof(raw->item_code));
    raw->nitem_code = 0;

	while(ofs < raw->len)
	{
		switch(p[ofs])
		{
            case kItemID:
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemID;
                raw->imu.id = U1(p+ofs+1);
                ofs += 2;
                break;
            case kItemAccRaw:													//加速度
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemAccRaw;
                raw->imu.acc[0] = (float)I2(p+ofs+1) / 1000;
                raw->imu.acc[1] = (float)I2(p+ofs+3) / 1000;
                raw->imu.acc[2] = (float)I2(p+ofs+5) / 1000;
                ofs += 7;
                break;
            case kItemGyrRaw:													//角速度
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemGyrRaw;
                raw->imu.gyr[0] = (float)I2(p+ofs+1) / 10;
                raw->imu.gyr[1] = (float)I2(p+ofs+3) / 10;
                raw->imu.gyr[2] = (float)I2(p+ofs+5) / 10;
                ofs += 7;
                break;
            case kItemMagRaw:													//磁强度
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemMagRaw;
                raw->imu.mag[0] = (float)I2(p+ofs+1) / 10;
                raw->imu.mag[1] = (float)I2(p+ofs+3) / 10;
                raw->imu.mag[2] = (float)I2(p+ofs+5) / 10;
                ofs += 7;
                break;
            case kItemRotationEul:										//欧拉角
                raw->item_code[raw->nitem_code++] = kItemRotationEul;
                raw->imu.eul[0] = (float)I2(p+ofs+1) / 100;
                raw->imu.eul[1] = (float)I2(p+ofs+3) / 100;
                raw->imu.eul[2] = (float)I2(p+ofs+5) / 10;
                ofs += 7;
                break;
            case kItemRotationQuat:										//四元数
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemRotationQuat;
                raw->imu.quat[0] = R4(p+ofs+1);
                raw->imu.quat[1] = R4(p+ofs+5);
                raw->imu.quat[2] = R4(p+ofs+9);
                raw->imu.quat[3] = R4(p+ofs+13);
                ofs += 17;
                break;
            case kItemPressure:												//气压
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemPressure;
                raw->imu.pressure = R4(p+ofs+1);
                ofs += 5;
                break;

            case KItemIMUSOL:
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = KItemIMUSOL;
                raw->imu.id = U1(p+ofs+1);
                raw->imu.pressure = R4(p+ofs+4);
                raw->imu.timestamp = U4(p+ofs+8);
                raw->imu.acc[0] = R4(p+ofs+12);
                raw->imu.acc[1] = R4(p+ofs+16);
                raw->imu.acc[2] = R4(p+ofs+20);
                raw->imu.gyr[0] = R4(p+ofs+24);
                raw->imu.gyr[1] = R4(p+ofs+28);
                raw->imu.gyr[2] = R4(p+ofs+32);
                raw->imu.mag[0] = R4(p+ofs+36);
                raw->imu.mag[1] = R4(p+ofs+40);
                raw->imu.mag[2] = R4(p+ofs+44);
                raw->imu.eul[0] = R4(p+ofs+48);
                raw->imu.eul[1] = R4(p+ofs+52);
                raw->imu.eul[2] = R4(p+ofs+56);
                raw->imu.quat[0] = R4(p+ofs+60);
                raw->imu.quat[1] = R4(p+ofs+64);
                raw->imu.quat[2] = R4(p+ofs+68);
                raw->imu.quat[3] = R4(p+ofs+72);
                ofs += 76;
                break;
				
            case KItemGWSOL:
                raw->item_code[raw->nitem_code++] = KItemGWSOL;
                raw->gwid = U1(p+ofs+1);
                raw->nimu = U1(p+ofs+2);
                ofs += 8;
                for (i=0; i<raw->nimu; i++)
                {
                    raw->imu.id = U1(p+ofs+1);
                    raw->imu.pressure = R4(p+ofs+4);
                    raw->imu.timestamp = U4(p+ofs+8);
                    raw->imu.acc[0] = R4(p+ofs+12);
                    raw->imu.acc[1] = R4(p+ofs+16);
                    raw->imu.acc[2] = R4(p+ofs+20);
                    raw->imu.gyr[0] = R4(p+ofs+24);
                    raw->imu.gyr[1] = R4(p+ofs+28);
                    raw->imu.gyr[2] = R4(p+ofs+32);
                    raw->imu.mag[0] = R4(p+ofs+36);
                    raw->imu.mag[1] = R4(p+ofs+40);
                    raw->imu.mag[2] = R4(p+ofs+44);
                    raw->imu.eul[0] = R4(p+ofs+48);
                    raw->imu.eul[1] = R4(p+ofs+52);
                    raw->imu.eul[2] = R4(p+ofs+56);
                    raw->imu.quat[0] = R4(p+ofs+60);
                    raw->imu.quat[1] = R4(p+ofs+64);
                    raw->imu.quat[2] = R4(p+ofs+68);
                    raw->imu.quat[3] = R4(p+ofs+72);
                    ofs += 76;
                }
                break;
            default:
				ofs++;
                break;
		}
    }
    
    return 1;
}

int decode_ch(raw_t *raw)
{
    uint16_t crc = 0;   

    /* checksum */
    crc16_update(&crc, raw->buf, 4);
    crc16_update(&crc, raw->buf+6, raw->len);
    if (crc != U2(raw->buf+4))
    {
        return -1;
    }
    
    return parse_data(raw);
}

/* sync code */
static int sync_ch(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

static int ch_serial_input(raw_t *raw, uint8_t data)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (!sync_ch(raw->buf, data)) return 0;
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;
    
    if (raw->nbyte == CH_HDR_SIZE)
    {
        if ((raw->len = U2(raw->buf+2)) > (MAXBUFLEN - CH_HDR_SIZE))
        {
            raw->nbyte = 0;
            return -1;
        }
    }
    
    if (raw->nbyte < (raw->len + CH_HDR_SIZE)) 
    {
        return 0;
    }
    
    raw->nbyte = 0;
    
    return decode_ch(raw);
}

