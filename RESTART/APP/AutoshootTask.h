#ifndef _AUTO_SHOOT_CONTROL_TASK_H_
#define _AUTO_SHOOT_CONTROL_TASK_H_
#include "main.h"

#define DEBUG_MODE 1

typedef struct
{
  float pitch;
  float yaw;
	uint8_t recogflag;
  int16_t receNewDataFlag;
  uint8_t crc;
} location;


typedef enum
{
  unkown = 0,
  blue = 1,
  red  = 2,
} robot_color_e;

//extern Speed_Prediction_t Speed_Prediction;
extern robot_color_e robot_color;
extern location new_location;
//extern uint8_t color_set_flag;
extern uint8_t auto_shoot_mode_set;
extern float yaw_buff;
void targetOffsetDataDeal(uint8_t  len, u8 *buf);

void process_general_message(unsigned char* address, unsigned int length);
void send_protocol(float pitch,float yaw);

#endif

