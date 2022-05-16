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
#ifndef __CH100_H
#define __CH100_H
#include "main.h"

#define CHSYNC1         (0x5A)        //message sync code 1  数据同步码1
#define CHSYNC2         (0xA5)        //message sync code 2  数据同步码2
#define CH_HDR_SIZE     (0x06)        //protocol header size 协议包头大小

#define MAXBUFLEN       (128)       /* max raw frame long */

/* data items */
#define kItemID                    (0x90)
#define kItemAccRaw                (0xA0)
#define kItemGyrRaw                (0xB0)
#define kItemMagRaw                (0xC0)
#define kItemRotationEul           (0xD0)
#define kItemRotationQuat          (0xD1)
#define kItemPressure              (0xF0)
#define KItemIMUSOL                (0x91)
#define KItemGWSOL                 (0x62)

typedef struct
{
    uint32_t id;            /* user defined ID       */
    float acc[3];           /* acceleration          */
    float gyr[3];           /* angular velocity      */  
    float mag[3];           /* magnetic field        */
    float eul[3];           /* attitude: eular angle */
    float quat[4];          /* attitude: quaternion  */
    float pressure;         /* air pressure          */
    uint32_t timestamp;
}ch_imu_data_t;

typedef struct
{
    int nbyte;                          /* number of bytes in message buffer */ 
    int len;                            /* message length (bytes) */
    uint8_t buf[MAXBUFLEN];             /* message raw buffer */
    uint8_t gwid;                       /* network ID(HI222) */
    uint8_t nimu;                       /* # of imu (HI222) */
    
    ch_imu_data_t imu;   								/* imu data list */
    uint8_t item_code[8];               /* item code recv in one frame */
    uint8_t nitem_code;                 /* # of item code */
}raw_t;

extern float pitch_Angle, yaw_Angle, roll_Angle; 
extern float pitch_Gyro, yaw_Gyro, roll_Gyro;
extern float x_Acc, y_Acc, z_Acc;
extern uint32_t imu_time_1ms;

void CH100_getDATA( void );
void USART6_Configuration_For_CH100( void );
int ch_serial_input(raw_t *raw, uint8_t data);
static void ch_dump_imu_data(raw_t *raw);


#endif
