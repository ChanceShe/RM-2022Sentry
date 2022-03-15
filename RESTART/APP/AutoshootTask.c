#include "main.h"
#define DEBUG_MODE 1

/* *** *** �Զ������ض��� ��ʼ *** *** */
uint8_t dataFromMF[7];		        //���ݻ���
uint8_t dataFromMFReadyFlag = 0; 		//�������ݽ�����ɱ�־λ

location new_location;            //������������ֵ

int sucflag = 0;    //�ɹ����յ�һ������
int recflag = 0;    //���ܵ�������ʶ��װ��
int receive = 0;    //���յ����ݸ���
/* *** *** ********************** *** *** */
int num_command=0;
uint8_t auto_shoot_mode_set;
robot_color_e robot_color = 1 ;   //0-9���±�ʶ�Լ����Ǻ췽��������������
char* command = "sc_r";
/********* �Զ������ض��� ���� *********/
int wExpected = 0;

Signal *Uart4_Protobuf_Receive_Message;
int16_t Uart4_Content_Size;
u8 *CRC_Content_buf;
TurretCommand *Uart4_Protobuf_Receive_Gimbal_Angle;
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
//      chassis.ctrl_mode=CHASSIS_REMOTE;
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
	Uart4_Protobuf_Receive_Gimbal_Angle->pitch = 0.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->yaw = 0.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->command = 0;
  Uart4_Protobuf_Receive_Gimbal_Angle=turret_command__unpack(NULL,content_length,content_address);
  flagg_pitch=Uart4_Protobuf_Receive_Gimbal_Angle->pitch;
  flagg_yaw=Uart4_Protobuf_Receive_Gimbal_Angle->yaw;
      /*���������⣬�����Ƿ���*/
	new_location.receNewDataFlag  =  1;
	if(Uart4_Protobuf_Receive_Gimbal_Angle->command == 1)
	{	
		  LASER_ON();
			new_location.x	=  -Uart4_Protobuf_Receive_Gimbal_Angle->yaw;
			new_location.y	=  -Uart4_Protobuf_Receive_Gimbal_Angle->pitch;
		  new_location.dis = Uart4_Protobuf_Receive_Gimbal_Angle->diatance;
			new_location.flag = 1;	
	}
	else
	{
		  LASER_OFF();
			new_location.x=  Uart4_Protobuf_Receive_Gimbal_Angle->yaw;
			new_location.y=  Uart4_Protobuf_Receive_Gimbal_Angle->pitch;
			new_location.flag = 0;	
			num_command++;		
	}

  turret_command__free_unpacked(Uart4_Protobuf_Receive_Gimbal_Angle,NULL);
}

typedef void(*Parser)(unsigned char* content_address, unsigned int content_length);

Parser parsers[] =
{
  NULL,
  &parse_signal,
  NULL,
  &parse_turret_command
};
unsigned int parsers_count = sizeof(parsers) / sizeof(Parser);

#ifndef DEBUG_MODE
#define DEBUG_MODE
#endif

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

  unsigned short package_id = *(unsigned short*)(&address[1]);
  unsigned short content_size = *(unsigned short*)(&address[3]);

  // Package tail check. 5 is the size of the head part.
  if (address[5 + content_size + 1] != 0xED) return;	//֡β0XED

#ifdef DEBUG_MODE
  ++counter_complete;
#endif

  unsigned char* content_address = &address[5];

  unsigned char crc8 = address[5 + content_size];

  if (crc8 != get_crc8(content_address, content_size)) 
		return;

#ifdef DEBUG_MODE
  ++counter_crc_passed;
#endif

  // Check id in order to prevent array out of range access.
  if (package_id < parsers_count)
    {
      (*parsers[package_id])(content_address,content_size);
    }
}



//void send_signal(char* name)
//{
//	/* example of using protobuf message */

//	Signal msg;
//	signal__init(&msg);
//	msg.name="612";
//	signal__pack(&msg,UART4_DMA_TX_BUF+5);
//
//	UART4_DMA_TX_BUF[0]=0xBE;
//	UART4_DMA_TX_BUF[1]=1;
//	UART4_DMA_TX_BUF[2]=0;
//	UART4_DMA_TX_BUF[3]=2;
//	UART4_DMA_TX_BUF[4]=0;
//
//	Append_CRC8_Check_Sum(&UART4_DMA_TX_BUF[5],4);
//	UART4_DMA_TX_BUF[9]=0xFE;
//
//}
// send_signal("atk_e");