#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "platform/time/dwt_delay.h"
#include "platform/time/delay.h"
#include "main.h"

DWT_Type test_dwt;
CoreDebug_Type test_core_debug;
uint32_t SystemCoreClock = 480000000U;
static uint32_t cycle_step;
static uint32_t cycle_reads;

uint32_t test_read_cycles(void)
{
    cycle_reads++;
    test_dwt.CYCCNT += cycle_step;
    return test_dwt.CYCCNT;
}

static void test_init_enables_cycle_counter(void)
{
    test_dwt = (DWT_Type){0};
    test_core_debug = (CoreDebug_Type){0};

    assert(DWT_Delay_Init());
    assert((test_core_debug.DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U);
    assert((test_dwt.CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
    assert(test_dwt.LAR == 0xC5ACCE55U);
    assert(test_dwt.CYCCNT == 0U);
}

static void test_delay_uses_core_clock_and_handles_wraparound(void)
{
    test_dwt.CYCCNT = UINT32_MAX - 100U;
    cycle_step = 240U;
    cycle_reads = 0U;

    DWT_DelayUs(2U);
    assert(cycle_reads >= 5U);
    assert(cycle_reads <= 7U);
}

static void test_legacy_delay_api_uses_dwt(void)
{
    cycle_step = 240U;
    cycle_reads = 0U;

    Delay_Init();
    Delay_us(1U);
    assert(cycle_reads >= 3U);
}

int main(void)
{
    test_init_enables_cycle_counter();
    test_delay_uses_core_clock_and_handles_wraparound();
    test_legacy_delay_api_uses_dwt();
    puts("DWT delay tests passed");
    return 0;
}
