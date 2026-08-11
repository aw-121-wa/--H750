#include "platform/time/delay.h"

#include "platform/time/dwt_delay.h"

void Delay_Init(void)
{
    (void)Time_Init();
}

void Delay_us(uint32_t microseconds)
{
    Time_DelayUs(microseconds);
}

void Delay_ms(uint16_t milliseconds)
{
    Time_DelayMsBlocking(milliseconds);
}
