#ifndef _AUTO_SHOOT_CONTROL_TASK_H_
#define _AUTO_SHOOT_CONTROL_TASK_H_
#include "main.h"

#define DEBUG_MODE 1

typedef struct
{
  float pitch;
  float yaw;
	int32_t distance;
	uint8_t recogflag;
  int16_t receNewDataFlag;
  uint8_t crc;
} location;

extern location new_location;
void targetOffsetDataDeal(uint8_t  len, u8 *buf);

void process_general_message(unsigned char* address, unsigned int length);
void send_protocol(float pitch,float yaw,uint8_t currentcolor);

#endif

