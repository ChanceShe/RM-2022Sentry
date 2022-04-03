#include "main.h"
#define DEBUG_MODE 1

/* *** *** 自动打击相关定义 开始 *** *** */
location new_location;            //传回来的坐标值
int receive = 0;    //接收到数据个数
/* *** *** *******************************/
int num_command=0;
uint8_t auto_shoot_mode_set;
robot_color_e robot_color = 1 ;   //0-9以下标识自己都是红方，其它都是蓝方
char* command = "sc_r";
/********* 自动打击相关定义 结束 *********/
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
	// Protobuf 正确用法是新建一个对象去存数据，就算不新建对象好歹初始化一下啊傻X刘恒
	Uart4_Protobuf_Receive_Gimbal_Angle->y = 0.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->x = 1.0f;
	Uart4_Protobuf_Receive_Gimbal_Angle->id = 0;
  Uart4_Protobuf_Receive_Gimbal_Angle=protocol__frame__unpack(NULL,content_length,content_address);
  flagg_pitch=Uart4_Protobuf_Receive_Gimbal_Angle->y;
  flagg_yaw=Uart4_Protobuf_Receive_Gimbal_Angle->x;
      /*这里有问题，数据是反的*/
	new_location.receNewDataFlag  =  1;
	if(Uart4_Protobuf_Receive_Gimbal_Angle->distance > 0)		//id颜色:0:篮,1:蓝,2:红,3:紫.
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


