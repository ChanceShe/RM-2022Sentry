#include "main.h"
#include "protocol.h"
//�������̽��ܣ�
/*
1����λ�����͹���У׼����
2������XXXCaliProcess,�����յ���У׼���ݱ��浽GyroCaliData��GimbalCaliData��MagCaliData����
3��У׼��ɺ󣬵���SetCaliData(*v)����У׼���ݱ��浽gAppParamStructFalsh��,��д��Flash��
*/
AppParam_t gAppParamStruct;	//������Ϣ,���ﱣ�������µ�У׼ֵ��������Flash�е�����ͬ��
static GyroCaliStruct_t GyroCaliData;        //����У׼������ƫ��ֵ
static GimbalCaliStruct_t  GimbalCaliData;   //������̨������ƫ��ֵ
static MagCaliStruct_t  MagCaliData;         //���������У׼ֵ
PIDParamStruct_t PIDCaliData;  //����pitch��positionУ׼ֵ
//�⼸������������ʵ�ʳ�����Ӧ��
GimbalCaliStruct_t GimbalSavedCaliData;    	    //gimbal pitch yaw encoder offset
GyroCaliStruct_t GyroSavedCaliData;     	    //gyro offset data
AccCaliStruct_t AccSavedCaliData;    	    	//ACC offset data
MagCaliStruct_t MagSavedCaliData;			    //Mag offset data
PIDParamStruct_t PitchPositionSavedPID;        	//PID offset data
PIDParamStruct_t PitchSpeedSavedPID;        	//PID offset data
PIDParamStruct_t YawPositionSavedPID;        	//PID offset data
PIDParamStruct_t YawSpeedSavedPID;        	    //PID offset data

uint8_t app_param_calied_flag = 0;


//������SuperviseTask�����ô����־λ
uint8_t Is_AppParam_Calied ( void )
{
    return app_param_calied_flag;    //paramδ��ʼ��
}

//���ڱ������ݵ�flash��
static uint8_t AppParamSave ( void )
{
    uint8_t retval = 1;
    retval = BSP_FLASH_Write ( PARAM_SAVED_START_ADDRESS, ( uint8_t * ) &gAppParamStruct, sizeof ( AppParam_t ) );
    if ( retval == 0 )
    {

    }
    return retval;
}

//�ֶ������������
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

void SetCaliCmdFlag ( uint32_t flag ) //����У׼��־λ
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







