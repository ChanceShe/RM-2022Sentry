#include "main.h"
#include "protocol.h"
//工作流程介绍：
/*
1、上位机发送过来校准参数
2、调用XXXCaliProcess,将接收到的校准数据保存到GyroCaliData、GimbalCaliData、MagCaliData等中
3、校准完成后，调用SetCaliData(*v)，将校准数据保存到gAppParamStructFalsh中,并写入Flash中
*/
AppParam_t gAppParamStruct;	//配置信息,这里保存着最新的校准值，并且与Flash中的内容同步
static GyroCaliStruct_t GyroCaliData;        //保存校准陀螺仪偏差值
static GimbalCaliStruct_t  GimbalCaliData;   //保存云台编码器偏差值
static MagCaliStruct_t  MagCaliData;         //保存磁力计校准值
PIDParamStruct_t PIDCaliData;  //保存pitch轴position校准值
//这几个变量用于在实际程序中应用
GimbalCaliStruct_t GimbalSavedCaliData;    	    //gimbal pitch yaw encoder offset
GyroCaliStruct_t GyroSavedCaliData;     	    //gyro offset data
AccCaliStruct_t AccSavedCaliData;    	    	//ACC offset data
MagCaliStruct_t MagSavedCaliData;			    //Mag offset data
PIDParamStruct_t PitchPositionSavedPID;        	//PID offset data
PIDParamStruct_t PitchSpeedSavedPID;        	//PID offset data
PIDParamStruct_t YawPositionSavedPID;        	//PID offset data
PIDParamStruct_t YawSpeedSavedPID;        	    //PID offset data

uint8_t app_param_calied_flag = 0;


//用于在SuperviseTask中设置错误标志位
uint8_t Is_AppParam_Calied ( void )
{
    return app_param_calied_flag;    //param未初始化
}

//用于保存数据到flash中
static uint8_t AppParamSave ( void )
{
    uint8_t retval = 1;
    retval = BSP_FLASH_Write ( PARAM_SAVED_START_ADDRESS, ( uint8_t * ) &gAppParamStruct, sizeof ( AppParam_t ) );
    if ( retval == 0 )
    {

    }
    return retval;
}

//手动设置相关数据
void AppParamInit ( void )
{

    GMPitchEncoder.ecd_bias = GMPitchEncoder_Offset;
    GMYawEncoder.ecd_bias = GMYawEncoder_Offset;
    GyroSavedCaliData.GyroXOffset = 0;
    GyroSavedCaliData.GyroYOffset = 0;
    GyroSavedCaliData.GyroZOffset = 0;
}



void SetGyroCaliData ( GyroCaliStruct_t *cali_data )
{
    if ( cali_data != NULL )
    {
        memcpy ( &gAppParamStruct.GyroCaliData, cali_data, sizeof ( *cali_data ) );
        AppParamSave();
    }
}

void SetAccCaliData ( AccCaliStruct_t *cali_data )
{
    if ( cali_data != NULL )
    {
        memcpy ( &gAppParamStruct.AccCaliData, cali_data, sizeof ( *cali_data ) );
        AppParamSave();
    }
}

void SetMagCaliData ( MagCaliStruct_t *cali_data )
{
    if ( cali_data != NULL )
    {
        memcpy ( &gAppParamStruct.MagCaliData, cali_data, sizeof ( *cali_data ) ); //step1: copy data to struct
        AppParamSave();
    }
    //step2:write data to the flash
}

//PID offset data saved in the memory
void SetPIDCaliData ( PIDParamStruct_t *cali_data )
{
    if ( cali_data != NULL )
    {
        if ( cali_data->pid_type == PID_TYPE_POSITION && cali_data->motor_type == MOTOR_TYPE_PITCH )
        {
            cali_data->kp_offset += gAppParamStruct.PitchPositionPID.kp_offset;
            cali_data->ki_offset += gAppParamStruct.PitchPositionPID.ki_offset;
            cali_data->kd_offset += gAppParamStruct.PitchPositionPID.kd_offset;
            memcpy ( &gAppParamStruct.PitchPositionPID, cali_data, sizeof ( *cali_data ) );
        }
        else if ( cali_data->pid_type == PID_TYPE_SPEED && cali_data->motor_type == MOTOR_TYPE_PITCH )
        {
            cali_data->kp_offset += gAppParamStruct.PitchSpeedPID.kp_offset;
            cali_data->ki_offset += gAppParamStruct.PitchSpeedPID.ki_offset;
            cali_data->kd_offset += gAppParamStruct.PitchSpeedPID.kd_offset;
            memcpy ( &gAppParamStruct.PitchSpeedPID, cali_data, sizeof ( *cali_data ) );
        }
        else if ( cali_data->pid_type == PID_TYPE_POSITION && cali_data->motor_type == MOTOR_TYPE_YAW )
        {
            cali_data->kp_offset += gAppParamStruct.YawPositionPID.kp_offset;
            cali_data->ki_offset += gAppParamStruct.YawPositionPID.ki_offset;
            cali_data->kd_offset += gAppParamStruct.YawPositionPID.kd_offset;
            memcpy ( &gAppParamStruct.YawPositionPID, cali_data, sizeof ( *cali_data ) );
        }
        else if ( cali_data->pid_type == PID_TYPE_SPEED && cali_data->motor_type == MOTOR_TYPE_YAW )
        {
            cali_data->kp_offset += gAppParamStruct.YawSpeedPID.kp_offset;
            cali_data->ki_offset += gAppParamStruct.YawSpeedPID.ki_offset;
            cali_data->kd_offset += gAppParamStruct.YawSpeedPID.kd_offset;
            memcpy ( &gAppParamStruct.YawSpeedPID, cali_data, sizeof ( *cali_data ) );
        }
        AppParamSave();
    }
}

void GetGimbalCaliData ( GimbalCaliStruct_t *cali_data )
{
    if ( cali_data != NULL )
    {
        memcpy ( cali_data, &gAppParamStruct.GimbalCaliData, sizeof ( GimbalCaliStruct_t ) );
    }
}

void GetGyroCaliData ( GyroCaliStruct_t *cali_data )
{
    if ( cali_data != NULL )
    {
        memcpy ( cali_data, &gAppParamStruct.GyroCaliData, sizeof ( GyroCaliStruct_t ) );
    }
}

void GetAccCaliData ( AccCaliStruct_t *cali_data )
{
    if ( cali_data != NULL )
    {
        memcpy ( cali_data, &gAppParamStruct.AccCaliData, sizeof ( AccCaliStruct_t ) );
    }
}

void GetMagCaliData ( MagCaliStruct_t *cali_data )
{
    if ( cali_data != NULL )
    {
        memcpy ( cali_data, &gAppParamStruct.MagCaliData, sizeof ( MagCaliStruct_t ) );
    }
}

uint8_t IsGimbalCalied ( void )
{
    return ( gAppParamStruct.GimbalCaliData.GimbalCaliFlag == PARAM_CALI_DONE );
}

uint8_t IsGyroCalied ( void )
{
    return ( gAppParamStruct.GyroCaliData.GyroCaliFlag == PARAM_CALI_DONE );
}

uint8_t IsAccCalied ( void )
{
    return ( gAppParamStruct.AccCaliData.AccCaliFlag == PARAM_CALI_DONE );
}

uint8_t IsMagCalied ( void )
{
    return ( gAppParamStruct.MagCaliData.MagCaliFlag == PARAM_CALI_DONE );
}

/*********************************************************************
 * @fn      CalibrateLoop
 *
 * @brief   do the calibration according to the corresponding cali flag
 *
 * @param   *flag_grp - the pointer to the cali flag group
 *
 * @return  none
 */


static uint32_t CaliCmdFlagGrp = 0;     //cali cmd flag group every bit represents a cali cmd received from the PC

void SetCaliCmdFlag ( uint32_t flag ) //设置校准标志位
{
    CaliCmdFlagGrp |= flag;
}

void ResetCaliCmdFlag ( uint32_t flag )
{
    CaliCmdFlagGrp &= ~flag;
}

uint32_t GetCaliCmdFlagGrp()
{
    return CaliCmdFlagGrp;
}

//to check whether a specfic flag if set
uint8_t IsCaliCmdFlagSet ( uint32_t flag )
{
    if ( flag & CaliCmdFlagGrp )
    {
        return 1;
    } else
    {
        return 0;
    }
}







