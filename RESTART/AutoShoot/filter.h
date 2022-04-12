#ifndef __FILTER_H
#define __FILTER_H
#include "main.h"

#define FILTER_NUM 5

float AvgFilter(float new_value);

extern float avgvalue[FILTER_NUM];

#endif


