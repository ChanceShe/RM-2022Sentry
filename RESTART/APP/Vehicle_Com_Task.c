/*************************************************************
 *  @file Vehicle_Com_Task.c
 *  @version 高校联盟赛 V1.0
 *  @date 2021.04.20

 *  @brief 车间通讯
 ************************************************************/
#include "main.h"

uint32_t receive_Dart;
uint32_t receive_Sentinel;
uint16_t Test_Info = 0;
reserve_aerial_info_t VehicleReserveInfo;
uint8_t VehicleShootFlagLow = 1;
uint8_t VehicleShootFlagUp = 1;
uint8_t VehicleGimbalFlag = 0;
void Vehicle_Send_Dart ( void )   //飞镖通讯为0x200
{
    //ID标识发送数据具体含义：0x200-0x2FF为学生端通信，此处发送为0x200，接收端需要对应
    ddata[0] = 0x00;
    ddata[1] = 0x02;

    //发送端机器人id，两个字节
    ddata[2] = ( judge_rece_mesg.game_robot_state.robot_id & 0x00FF );
    ddata[3] = ( judge_rece_mesg.game_robot_state.robot_id & 0xFF00 ) >> 8;

    //目标机器人id，两个字节
    if ( judge_rece_mesg.game_robot_state.robot_id < 10 ) //红方
    {
        ddata[4] = 0x08;     //0x08为红方飞镖架
        ddata[5] = 0x00;
    }
    else
    {
        ddata[4] = 0x6C;     //0x6C为蓝方飞镖架,十进制108
        ddata[5] = 0x00;
    }

    Test_Info ++ ;

    ddata[6] = ( Test_Info & 0x00FF );
    ddata[7] = ( Test_Info & 0xFF00 ) >> 8;

    ddata[8] = 11;
    ddata[9] = 11 >> 8;

    //*(ext_client_custom_graphic_seven_t*)(&ddata[6]) = 32;
    //data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(0x02),DN_REG_ID,tx_buf);

    data_upload_handle ( STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata, 10, DN_REG_ID, tx_buf );
    //TUDENT_INTERACTIVE_HEADER_DATA_ID为裁判系统接收的ID
    //ddata为发送的数据（包含各种学生端通信需要的头id）
    //sizeof(ddata)-6为发送的(除了帧头ID的)数据段数据量  或者是8
    //DN_REG_ID为裁判系统接收的头帧
    //发送的数据处理保存至tx_buf
}

void Vehicle_Send_Sentinel ( void ) //哨兵通讯为0x201
{
    //ID标识发送数据具体含义：0x200-0x2FF为学生端通信，此处发送为0x201，接收端需要对应
    ddata[0] = 0x01;
    ddata[1] = 0x02;

    //发送端机器人id，两个字节
    ddata[2] = ( judge_rece_mesg.game_robot_state.robot_id & 0x00FF );
    ddata[3] = ( judge_rece_mesg.game_robot_state.robot_id & 0xFF00 ) >> 8;

    //目标机器人id，两个字节
    if ( judge_rece_mesg.game_robot_state.robot_id < 10 ) //红方
    {
        ddata[4] = 0x06;     //0x07为红方哨兵
        ddata[5] = 0x00;
    }
    else
    {
        ddata[4] = 0x6A;     //0x6B为蓝方哨兵,十进制107
        ddata[5] = 0x00;
    }

    Test_Info ++ ;
    ddata[6] = robot_position;
    ddata[7] = ( judge_rece_mesg.bullet_remaining.bullet_remaining_num_17mm & 0x00FF );
    ddata[8] = ( judge_rece_mesg.bullet_remaining.bullet_remaining_num_17mm & 0xFF00 ) >> 8;
//    ddata[9] = Test_Info;

    //*(ext_client_custom_graphic_seven_t*)(&ddata[6]) = 32;
    //data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(0x02),DN_REG_ID,tx_buf);

    data_upload_handle ( STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata, 9, DN_REG_ID, tx_buf );
    //TUDENT_INTERACTIVE_HEADER_DATA_ID为裁判系统接收的ID
    //ddata为发送的数据（包含各种学生端通信需要的头id）
    //sizeof(ddata)-6为发送的(除了帧头ID的)数据段数据量  或者是8
    //DN_REG_ID为裁判系统接收的头帧
    //发送的数据处理保存至tx_buf
}

void Vehicle_Receive ( void )
{
    if ( judge_rece_mesg.student_interactive_header_data.data_cmd_id == 0x200 )  //飞镖信息
    {
        if ( ( judge_rece_mesg.game_robot_state.robot_id == judge_rece_mesg.student_interactive_header_data.receiver_ID )
                && ( judge_rece_mesg.student_interactive_header_data.send_ID == 4
                     ||  judge_rece_mesg.student_interactive_header_data.send_ID == 104 ) )    //发送端4  步兵
        {
            receive_Dart = ( judge_rece_mesg.robot_interactive_data.data[0] )    | ( judge_rece_mesg.robot_interactive_data.data[1] << 8 ) |
                           ( judge_rece_mesg.robot_interactive_data.data[2] << 16 ) | ( judge_rece_mesg.robot_interactive_data.data[3] << 24 );
        }
    }

    else if ( judge_rece_mesg.student_interactive_header_data.data_cmd_id == 0x201 ) //哨兵信息
    {
        if ( ( judge_rece_mesg.game_robot_state.robot_id == judge_rece_mesg.student_interactive_header_data.receiver_ID )
                && ( judge_rece_mesg.student_interactive_header_data.send_ID == 1
                     ||  judge_rece_mesg.student_interactive_header_data.send_ID == 101 ) )    //发送端1  英雄
        {
            receive_Sentinel = ( judge_rece_mesg.robot_interactive_data.data[0] )    | ( judge_rece_mesg.robot_interactive_data.data[1] << 8 ) |
                               ( judge_rece_mesg.robot_interactive_data.data[2] << 16 ) | ( judge_rece_mesg.robot_interactive_data.data[3] << 24 );
        }
    }
}

void vehicle_control_chassis ( void )
{
    if ( VehicleReserveInfo.chassis_state == 1 || VehicleReserveInfo.chassis_state == 2 )
    {
//        chassis.ctrl_mode = CHASSIS_TEST;
    }
    else
    {
        chassis.ctrl_mode = CHASSIS_PATROL;
    }
    
}

void vehicle_control_gimbal ( void )
{
    
    if ( VehicleReserveInfo.upper_shoot_state == 1 )
    {
        VehicleShootFlagUp = 0;
    }
    else
    {
        VehicleShootFlagUp = 1;
    }

    if ( VehicleReserveInfo.lower_shoot_state == 1 )
    {
        VehicleShootFlagLow = 0;
    }
    else
    {
        VehicleShootFlagLow = 1;
    }

//    if ( VehicleReserveInfo.lower_gimbal_state == 1 || VehicleReserveInfo.lower_gimbal_state == 2)
//    {
//        VehicleGimbalFlag = 1;
//    }
//    else
//    {
//        VehicleGimbalFlag = 0;
//    }

}

void Vehicle_Com_Task ( void )
{
//   Vehicle_Send_Sentinel();
//   Vehicle_Send_Dart();
//	Vehicle_Receive();
}

