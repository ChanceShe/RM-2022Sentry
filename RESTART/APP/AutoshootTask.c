#include "main.h"


/* *** *** 自动打击相关定义 开始 *** *** */
location new_location;            //传回来的坐标值
int receive_time = 0;
int last_receive_time = 0;

/* *** *** *******************************/
uint8_t auto_shoot_mode_set;
robot_color_e robot_color = unkown ;   //0-9以下标识自己都是红方，其它都是蓝方
char* command = "sc_r";
/********* 自动打击相关定义 结束 *********/
int wExpected = 0;

u8 *CRC_Content_buf;
VToE__Frame *Uart4_Protobuf_Receive_Gimbal_Angle;
//extern unsigned char get_crc8(unsigned char* data, unsigned int length);
void targetOffsetDataDeal ( uint8_t len, u8 *buf )
{
  process_general_message(buf,len);
}


float yaw_buff=0;

void parse_turret_command(unsigned char* content_address, unsigned int content_length)
{
	// Protobuf 正确用法是新建一个对象去存数据，就算不新建对象好歹初始化一下啊傻X刘恒
	Uart4_Protobuf_Receive_Gimbal_Angle->targetpitch_ = 0.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->targetyaw_ = 0.0f;
  Uart4_Protobuf_Receive_Gimbal_Angle=v_to_e__frame__unpack(NULL,content_length,content_address);
      /*这里有问题，数据是反的*/
	new_location.receNewDataFlag  =  1;
	if(Uart4_Protobuf_Receive_Gimbal_Angle->targetpitch_ != 0&&Uart4_Protobuf_Receive_Gimbal_Angle->targetyaw_ != 0)		//id颜色:0:篮,1:蓝,2:红,3:紫.
	{	
		  LASER_ON();
			new_location.pitch			= Uart4_Protobuf_Receive_Gimbal_Angle->targetyaw_;
			new_location.yaw				= Uart4_Protobuf_Receive_Gimbal_Angle->targetpitch_;
			new_location.recogflag	= 1;
	}
	else
	{
		  LASER_OFF();
			new_location.pitch				= 0;
			new_location.yaw					= 0;
			new_location.recogflag		= 0;
	}
 
  v_to_e__frame__free_unpacked(Uart4_Protobuf_Receive_Gimbal_Angle,NULL);
}
\
typedef void(*Parser)(unsigned char* content_address, unsigned int content_length);

Parser parsers[] =
{
  NULL,
  NULL,
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
  if (address[0] != 0xBE) return;		//帧头0XBE

  unsigned short content_size = 0;	//数据段长度
	//unsigned short package_id = *(unsigned short*)(&address[1]);		//自瞄模式: 1打符 3自瞄
  // Package tail check. 2 is the size of the head part.
	for(content_size = 0; 2+content_size  < length; content_size++)
	{
		if(address[2+content_size] == 0xED && address[2+content_size+1] == 0X0D && address[2+content_size+2] == 0X0A) //帧尾0XED 0X0D 0X0A
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

EToV__Frame msg;
char *flagg;
u8 DateLength;

void send_protocol(float pitch,float yaw)
{
  e_to_v__frame__init(&msg);
  msg.currentpitch_=(int)(pitch*100);
	msg.currentyaw_=(int)(yaw*100);
	msg.currentcolor_= 101;
	msg.bulletspeed_=27*10;
  e_to_v__frame__pack(&msg,UART4_DMA_TX_BUF+1);
  DateLength=e_to_v__frame__get_packed_size(&msg);
  UART4_DMA_TX_BUF[0]=0xBE;
  Append_CRC8_Check_Sum(&UART4_DMA_TX_BUF[1],DateLength+1);
  UART4_DMA_TX_BUF[DateLength+2]=0xED;
	UART4_DMA_TX_BUF[DateLength+3]='\r';
	UART4_DMA_TX_BUF[DateLength+4]='\n';
	Uart4SendBytesInfoProc(UART4_DMA_TX_BUF,DateLength+5);
}

