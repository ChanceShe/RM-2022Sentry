#include "filter.h"

float avgvalue[FILTER_NUM]={0,0,0,0,0};

float AvgFilter(float new_value)//均值滤波，用于自瞄
{
	float avg_value;
	float sum = 0;
	for(int i = 0; i < FILTER_NUM - 1 ; i ++)
	{
			avgvalue[i] = avgvalue[i + 1];
			sum += avgvalue[i];
	}
	avgvalue[FILTER_NUM - 1] = new_value;
	sum += avgvalue[FILTER_NUM - 1];
	avg_value = sum / FILTER_NUM;
	return avg_value;

}
