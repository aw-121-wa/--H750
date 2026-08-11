#ifndef PLATFORM_TIME_DWT_DELAY_H
#define PLATFORM_TIME_DWT_DELAY_H

#include <stdbool.h>
#include <stdint.h>

bool Time_Init(void);
void Time_DelayUs(uint32_t microseconds);
void Time_DelayMsBlocking(uint32_t milliseconds);

#define DWT_Delay_Init Time_Init
#define DWT_DelayUs    Time_DelayUs

#endif
