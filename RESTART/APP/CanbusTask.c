#include "main.h"

static uint32_t can1_count = 0;
static uint32_t can2_count = 0;

//CAN1���������
volatile Encoder PokeEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};

//CAN2���������
volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};								//����������
volatile Encoder GMYawEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};			//����̨Yaw
volatile Encoder GMPitchEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};		//����̨Pitch
volatile Encoder Friction1Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};		//����̨Pitch
volatile Encoder Friction2Encoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};		//����̨Pitch




void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg) 
{
  v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //�����ʼ������ֵ��Ϊƫ��  
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
	if(v->diff < -4096)    //���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
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
	//����õ������ı��������ֵ
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//����õ��Ƕ�ֵ����Χ���������
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//�����ٶ�ƽ��ֵ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}

void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
	can1_count++;
	switch(msg->StdId)
  {
		case CAN_BUS1_CHASSIS_MOTOR_FEEDBACK_MSG_ID:		//����������
		{
			LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR1 ) );
			( can2_count <= 50 ) ? GetEncoderBias ( &CM1Encoder , msg ) : EncoderProcess ( &CM1Encoder , msg ); //��ȡ���������ĳ�ʼƫ��ֵ
		}
		break;
	}
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{      
    can2_count++;
		switch(msg->StdId)
		{

			case CAN_BUS2_PITCH_MOTOR_FEEDBACK_MSG_ID:    //��̨�������
      {
         LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR6 ) );
         //GMPitchEncoder.ecd_bias = pitch_ecd_bias;
         EncoderProcess ( &GMPitchEncoder , msg );
         //�����м�ֵ�趨Ҳ��Ҫ�޸�
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
			case CAN_BUS2_YAW_MOTOR_FEEDBACK_MSG_ID ://��̨�������
			{
					EncoderProcess ( &GMYawEncoder , msg );
					LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR5 ) );
					//GMYawEncoder.ecd_bias = yaw_ecd_bias;
					// �Ƚϱ����������ֵ��ƫ��ֵ�������������ֵ�ͳ�ʼƫ��֮���೬����ֵ����ƫ��ֵ��������ֹ������̨�������˶�
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
			case CAN_BUS2_POKE_FEEDBACK_MSG_ID :   //�������
			{
				 //LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
				 ( can1_count <= 50 ) ? GetEncoderBias ( &PokeEncoder , msg ) : EncoderProcess ( &PokeEncoder , msg );
			}
			break;

		  case CAN_BUS2_FRICTION_MOTOR1_FEEDBACK_MSG_ID:
      {
          LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR2 ) );
          ( can2_count <= 50 ) ? GetEncoderBias ( &Friction1Encoder , msg ) : EncoderProcess ( &Friction1Encoder , msg );
      }
      break;
      case CAN_BUS2_FRICTION_MOTOR2_FEEDBACK_MSG_ID:
      {
          LostCounterFeed ( GetLostCounter ( LOST_COUNTER_INDEX_MOTOR2 ) );
          ( can2_count <= 50 ) ? GetEncoderBias ( &Friction2Encoder , msg ) : EncoderProcess ( &Friction2Encoder , msg );
      }
      break;

			default:
			{
			}
			break;
		}	
		LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
}

void CAN1_Send_Msg(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)//CAN1���ͺ��� ǰ�ĸ�ID
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
void CAN1_Send_Msg1(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq) //CAN1���ͺ��� ���ĸ�ID
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


void CAN2_Send_Msg(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq) //CAN2���ͺ��� ǰ�ĸ�ID
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
void CAN2_Send_Msg1(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq) //CAN2���ͺ��� ���ĸ�ID
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

void upperboard_send_to_mainboard ( CAN_TypeDef *CANx, uint16_t ch2, uint16_t ch3, uint8_t s1, uint8_t s2 , uint8_t coler,
                                    uint16_t shoot_heart, uint8_t VehicleShootFlag, uint8_t JudgeShootFlag )							//���°�ͨѶ
{
    CanTxMsg tx_message;

    tx_message.StdId = 0x408;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    tx_message.Data[0] = ( uint8_t ) ( ch2 >> 8 );
    tx_message.Data[1] = ( uint8_t ) ch2;
    tx_message.Data[2] = ( uint8_t ) ( ch3 >> 8 );
    tx_message.Data[3] = ( uint8_t ) ch3;
    tx_message.Data[4] = ( uint8_t ) ( s1 << 4 ) | s2;
    tx_message.Data[5] = ( uint8_t ) ( coler << 7 ) | ( VehicleShootFlag << 6 ) | JudgeShootFlag ;
    tx_message.Data[6] = ( uint8_t ) ( shoot_heart >> 8 );
    tx_message.Data[7] = ( uint8_t ) shoot_heart;

    CAN_Transmit ( CANx, &tx_message );
}

