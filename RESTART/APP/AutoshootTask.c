#include "main.h"
#define DEBUG_MODE 1

/* *** *** �Զ������ض��� ��ʼ *** *** */
location new_location;            //������������ֵ
int receive = 0;    //���յ����ݸ���
/* *** *** *******************************/
int num_command=0;
uint8_t auto_shoot_mode_set;
robot_color_e robot_color = 1 ;   //0-9���±�ʶ�Լ����Ǻ췽��������������
char* command = "sc_r";
/********* �Զ������ض��� ���� *********/
int wExpected = 0;

Signal *Uart4_Protobuf_Receive_Message;
int16_t Uart4_Content_Size;
u8 *CRC_Content_buf;
Protocol__Frame *Uart4_Protobuf_Receive_Gimbal_Angle;
//extern unsigned char get_crc8(unsigned char* data, unsigned int length);
float flagg_pitch;
float flagg_yaw;
void targetOffsetDataDeal ( uint8_t len, u8 *buf )
{
  receive++;
  process_general_message(buf,len);
}


void parse_signal(unsigned char* content_address, unsigned int content_length)
{
  //Todo: parse and process signal.
  Uart4_Protobuf_Receive_Message=signal__unpack(NULL,content_length,content_address);
  if(strcmp(Uart4_Protobuf_Receive_Message->name,"t_c_d"))
    {
    }
  else if(strcmp(Uart4_Protobuf_Receive_Message->name,"t_c_a"))
    {
      gim.ctrl_mode=GIMBAL_INIT;
    }
  signal__free_unpacked(Uart4_Protobuf_Receive_Message,NULL);
}

float yaw_buff=0;

void parse_turret_command(unsigned char* content_address, unsigned int content_length)
{
	// Protobuf ��ȷ�÷����½�һ������ȥ�����ݣ����㲻�½�����ô���ʼ��һ�°�ɵX����
	Uart4_Protobuf_Receive_Gimbal_Angle->y = 0.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->x = 1.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->id = 0;
  Uart4_Protobuf_Receive_Gimbal_Angle=protocol__frame__unpack(NULL,content_length,content_address);
  flagg_pitch=Uart4_Protobuf_Receive_Gimbal_Angle->y;
  flagg_yaw=Uart4_Protobuf_Receive_Gimbal_Angle->x;
      /*���������⣬�����Ƿ���*/
	new_location.receNewDataFlag  =  1;
	if(Uart4_Protobuf_Receive_Gimbal_Angle->distance > 0)		//id��ɫ:0:��,1:��,2:��,3:��.
	{	
		  LASER_ON();
			new_location.x		= Uart4_Protobuf_Receive_Gimbal_Angle->x;
			new_location.y		= Uart4_Protobuf_Receive_Gimbal_Angle->y;
		  new_location.dis	= Uart4_Protobuf_Receive_Gimbal_Angle->distance;
			new_location.id		= Uart4_Protobuf_Receive_Gimbal_Angle->color;
	}
	else
	{
		  LASER_OFF();
			new_location.x=  Uart4_Protobuf_Receive_Gimbal_Angle->x;
			new_location.y=  Uart4_Protobuf_Receive_Gimbal_Angle->y;
			num_command++;		
	}

  protocol__frame__free_unpacked(Uart4_Protobuf_Receive_Gimbal_Angle,NULL);
}

typedef void(*Parser)(unsigned char* content_address, unsigned int content_length);

Parser parsers[] =
{
  NULL,
  &parse_signal,
  NULL,
  &parse_turret_command
};


#ifdef DEBUG_MODE
  // Counter for received packages.
  int counter_receive = 0;
  // Counter for packages that passed the head and tail check.
  int counter_complete = 0;
  // Contuner for pacakages that passed the CRC8 check.
  int counter_crc_passed = 0;
#endif


void process_general_message(unsigned char* address, unsigned int length)
{

#ifdef DEBUG_MODE
  if (length > 0) ++counter_receive;
#endif

  // Package head check.
  if (address[0] != 0xBE) return;		//֡ͷ0XBE

  unsigned short content_size = 0;	//���ݶγ���
	//unsigned short package_id = *(unsigned short*)(&address[1]);		//����ģʽ: 1��� 3����
  // Package tail check. 2 is the size of the head part.
	for(content_size = 0; 2+content_size  < length; content_size++)
	{
		if(address[2+content_size] == 0xED && address[2+content_size+1] == 0X0D && address[2+content_size+2] == 0X0A) //֡β0XED 0X0D 0X0A
			break;
		else if(2+content_size+1 > length) 
			return;
	}

#ifdef DEBUG_MODE
  ++counter_complete;
#endif
  // CRC check.
  unsigned char* content_address = &address[1];

  unsigned char crc8 = address[1 + content_size];

  if (crc8 != get_crc8(content_address, content_size)) 
		return;

#ifdef DEBUG_MODE
  ++counter_crc_passed;
#endif
	(*parsers[3])(content_address,content_size);
}


