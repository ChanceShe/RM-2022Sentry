#include "main.h"

static uint32_t can1_count = 0;
static uint32_t can2_count = 0;

//CAN1电机编码器
volatile Encoder Friction1Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};		//云台摩擦轮
volatile Encoder Friction2Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};		//云台摩擦轮
volatile Encoder Friction3Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};		//云台摩擦轮
volatile Encoder Friction4Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};		//云台摩擦轮

//CAN2电机编码器
volatile Encoder GMYawEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};				//云台Yaw
volatile Encoder GMPitchEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};			//云台Pitch
volatile Encoder Poke1Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};					//拨盘
volatile Encoder Poke2Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};					//拨盘

//refrom_info_t  main_info = {0};
refrom_mainboard_t refromData = {0, 0, 0, 0};
uint8_t currentid = 0;

static volatile Shoot_State_e shootState_h = NOSHOOTING;
uint8_t  ShootConFlag = 1;


void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg) 
{
  v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
  v->ecd_value = v->ecd_bias;
  v->last_raw_value = v->ecd_bias;
  v->temp_count++;
}

void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}

void revice_main_information ( refrom_mainboard_t *data, CanRxMsg * msg )
{
    data->ch2 = ( msg->Data[0] << 8 ) | msg->Data[1];
    data->ch3 = ( msg->Data[2] << 8 ) | msg->Data[3];
    data->s1 = msg->Data[4] >> 4;
    data->s2 = msg->Data[4] & 0x0f;
    data->color = msg->Data[5] >> 7 & 0x00000001;
    data->VehicleShootFlag = ( msg->Data[5] >> 6 ) & 0x00000001;
    data->JudgeShootFlag = msg->Data[5] & 0x03;
    data->shoot_heart_l =  msg->Data[6];
		data->shoot_heart_r =  msg->Data[7];
}

void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
	can1_count++;
	switch(msg->StdId)
  {
		
		case CAN_BUS1_FRICTION_MOTOR1_FEEDBACK_MSG_ID:		//摩擦轮电机处理
		{
				LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR1 ) );
				( can2_count <= 50 ) ? GetEncoderBias ( &Friction1Encoder , msg ) : EncoderProcess ( &Friction1Encoder , msg );
		}
		break;
		case CAN_BUS1_FRICTION_MOTOR2_FEEDBACK_MSG_ID:		//摩擦轮电机处理
		{
				LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR2 ) );
				( can2_count <= 50 ) ? GetEncoderBias ( &Friction2Encoder , msg ) : EncoderProcess ( &Friction2Encoder , msg );
		}
		break;
		case CAN_BUS1_FRICTION_MOTOR3_FEEDBACK_MSG_ID:		//摩擦轮电机处理
		{
				LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR3 ) );
				( can2_count <= 50 ) ? GetEncoderBias ( &Friction3Encoder , msg ) : EncoderProcess ( &Friction3Encoder , msg );
		}
		break;
		case CAN_BUS1_FRICTION_MOTOR4_FEEDBACK_MSG_ID:		//摩擦轮电机处理
		{
				LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR4 ) );
				( can2_count <= 50 ) ? GetEncoderBias ( &Friction4Encoder , msg ) : EncoderProcess ( &Friction4Encoder , msg );
		}
		break;
		default:
		break;
	}
}
void Can2ReceiveMsgProcess(CanRxMsg * msg)
{      
    can2_count++;
		switch(msg->StdId)
		{
			case CAN_BUS2_PITCH_MOTOR_FEEDBACK_MSG_ID:    //云台电机处理
			{
				 LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR6 ) );
				 //GMPitchEncoder.ecd_bias = pitch_ecd_bias;
				 EncoderProcess ( &GMPitchEncoder , msg );
				 //码盘中间值设定也需要修改
				 if ( can2_count >= 90 && can2_count <= 100 )
				 {
						if ( ( GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value ) < -4096 )
						{
								GMPitchEncoder.ecd_bias = GMPitchEncoder_Offset + 8192;
						}
						else if ( ( GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value ) > 4096 )
						{
								GMPitchEncoder.ecd_bias = GMPitchEncoder_Offset - 8192;
						}
				 }
			}
			break;
			case CAN_BUS2_YAW_MOTOR_FEEDBACK_MSG_ID ://云台电机处理
			{
					EncoderProcess ( &GMYawEncoder , msg );
					LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR5 ) );
					// 比较保存编码器的值和偏差值，如果编码器的值和初始偏差之间差距超过阈值，将偏差值做处理，防止出现云台反方向运动
					if ( can2_count >= 90 && can2_count <= 100 )
					{
						 if ( ( GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value ) < -4096 )
						 {
								 GMYawEncoder.ecd_bias = GMYawEncoder_Offset + 8192;
						 }
						 else if ( ( GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value ) > 4096 )
						 {
								 GMYawEncoder.ecd_bias = GMYawEncoder_Offset - 8192;
						 }
					}
			}
			break;
			case CAN_BUS2_POKE1_FEEDBACK_MSG_ID :   //拨弹电机
			{
				 ( can1_count <= 50 ) ? GetEncoderBias ( &Poke1Encoder , msg ) : EncoderProcess ( &Poke1Encoder , msg );
			}
			break;
			case CAN_BUS2_POKE2_FEEDBACK_MSG_ID :   //拨弹电机
			{
				 ( can1_count <= 50 ) ? GetEncoderBias ( &Poke2Encoder , msg ) : EncoderProcess ( &Poke2Encoder , msg );
			}
			break;
			case CAN_BUS2_SLAVE_FEEDBACK_MSG_ID:                  //上下云台通信
      {
            LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_ZGYRO ) );
            revice_main_information ( &refromData, msg );
            if ( refromData.color == 0 )     //颜色切换
            {
                currentid = 7;
            }
            else if ( refromData.color == 1 )
            {
                currentid = 107;
            }
						RemoteDataPrcess(&refromData);
			}
      break;
			default:
			{
			}
			break;
		}	
		LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
}

void CAN1_Send_Msg(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)//CAN1发送函数 前四个ID
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
}
void CAN1_Send_Msg1(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq) //CAN1发送函数 后四个ID
{
    CanTxMsg tx_message;
    tx_message.StdId =0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(cm5_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm5_iq;
    tx_message.Data[2] = (uint8_t)(cm6_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm6_iq;
    tx_message.Data[4] = (uint8_t)(cm7_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm7_iq;
    tx_message.Data[6] = (uint8_t)(cm8_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm8_iq;
    CAN_Transmit(CANx,&tx_message);
}

void CAN2_Send_Msg(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq) //CAN2发送函数 前四个ID
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
}
void CAN2_Send_Msg1(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq) //CAN2发送函数 后四个ID
{
    CanTxMsg tx_message;
    tx_message.StdId =0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(cm5_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm5_iq;
    tx_message.Data[2] = (uint8_t)(cm6_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm6_iq;
    tx_message.Data[4] = (uint8_t)(cm7_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm7_iq;
    tx_message.Data[6] = (uint8_t)(cm8_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm8_iq;
    CAN_Transmit(CANx,&tx_message);
}
void CAN2_Gimbal_Msg ( int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq )
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    tx_message.Data[0] = ( unsigned char ) ( gimbal_pitch_iq >> 8 );
    tx_message.Data[1] = ( unsigned char ) gimbal_pitch_iq;
    tx_message.Data[2] = ( unsigned char ) ( gimbal_yaw_iq >> 8 );
    tx_message.Data[3] = ( unsigned char ) gimbal_yaw_iq;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit ( CAN2, &tx_message );
}
