#ifndef __VEHICLE_COM_TASK_H__
#define __VEHICLE_COM_TASK_H__
#include "main.h"

typedef struct
{
    uint8_t chassis_state;
    uint8_t lower_shoot_state;
    uint8_t upper_shoot_state;
    uint8_t lower_gimbal_state;
    uint8_t return_flag;
    uint8_t test_info;
} reserve_aerial_info_t;


extern reserve_aerial_info_t VehicleReserveInfo;
extern uint8_t VehicleShootFlagLow;
extern uint8_t VehicleShootFlagUp;
extern uint8_t VehicleGimbalFlag;
void Vehicle_Com_Task ( void );
void Vehicle_Receive ( void );
void Vehicle_Send_Sentinel ( void );
void Vehicle_Send_Dart ( void );
void vehicle_control_chassis ( void );
void vehicle_control_gimbal ( void );
#endif


