#ifndef PLATFORM_TIME_DELAY_H
#define PLATFORM_TIME_DELAY_H

#include <stdint.h>

void Delay_Init(void);
void Delay_us(uint32_t microseconds);
void Delay_ms(uint16_t milliseconds);

#endif
