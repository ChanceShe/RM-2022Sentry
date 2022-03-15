#include "main.h"
#define DEBUG_MODE 1

/* *** *** 自动打击相关定义 开始 *** *** */
uint8_t dataFromMF[7];		        //数据缓存
uint8_t dataFromMFReadyFlag = 0; 		//妙算数据接收完成标志位

location new_location;            //传回来的坐标值

int sucflag = 0;    //成功接收到一组数据
int recflag = 0;    //接受到的数据识别到装甲
int receive = 0;    //接收到数据个数
/* *** *** ********************** *** *** */
int num_command=0;
uint8_t auto_shoot_mode_set;
robot_color_e robot_color = 1 ;   //0-9以下标识自己都是红方，其它都是蓝方
char* command = "sc_r";
/********* 自动打击相关定义 结束 *********/
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
	// Protobuf 正确用法是新建一个对象去存数据，就算不新建对象好歹初始化一下啊傻X刘恒
	Uart4_Protobuf_Receive_Gimbal_Angle->pitch = 0.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->yaw = 0.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->command = 0;
  Uart4_Protobuf_Receive_Gimbal_Angle=turret_command__unpack(NULL,content_length,content_address);
  flagg_pitch=Uart4_Protobuf_Receive_Gimbal_Angle->pitch;
  flagg_yaw=Uart4_Protobuf_Receive_Gimbal_Angle->yaw;
      /*这里有问题，数据是反的*/
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
  if (address[0] != 0xBE) return;		//帧头0XBE

  unsigned short package_id = *(unsigned short*)(&address[1]);
  unsigned short content_size = *(unsigned short*)(&address[3]);

  // Package tail check. 5 is the size of the head part.
  if (address[5 + content_size + 1] != 0xED) return;	//帧尾0XED

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