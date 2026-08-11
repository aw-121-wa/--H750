#include "platform/time/dwt_delay.h"

#include <limits.h>

#include "main.h"

#ifndef DWT_DELAY_CYCLE_COUNTER
#define DWT_DELAY_CYCLE_COUNTER Time_ReadCycleCounter
static inline uint32_t Time_ReadCycleCounter(void)
{
    return DWT->CYCCNT;
}
#endif

bool Time_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55U;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}

void Time_DelayUs(uint32_t microseconds)
{
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    const uint32_t max_chunk_us = cycles_per_us == 0U ?
                                  0U : UINT32_MAX / cycles_per_us;

    while (microseconds > 0U && max_chunk_us > 0U)
    {
        const uint32_t chunk_us = microseconds > max_chunk_us ?
                                  max_chunk_us : microseconds;
        const uint32_t wait_cycles = chunk_us * cycles_per_us;
        const uint32_t start = DWT_DELAY_CYCLE_COUNTER();

        while ((uint32_t)(DWT_DELAY_CYCLE_COUNTER() - start) < wait_cycles)
        {
        }
        microseconds -= chunk_us;
    }
}

void Time_DelayMsBlocking(uint32_t milliseconds)
{
    while (milliseconds > 0U)
    {
        const uint32_t chunk_ms = milliseconds > 1000U ? 1000U : milliseconds;
        Time_DelayUs(chunk_ms * 1000U);
        milliseconds -= chunk_ms;
    }
}
