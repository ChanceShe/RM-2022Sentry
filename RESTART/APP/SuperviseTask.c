#include "main.h"
LostCounter_t lost_counter[LOST_COUNTER_NUM] = { 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000,2000};

uint32_t *GetLostCounter(uint8_t index)
{
    if(index < LOST_COUNTER_NUM)
    {
        return &lost_counter[index];
    }
    else
    {
        return ((void *)0);
    } 
}
//return the error code
