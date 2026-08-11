#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "app/diagnostics/chassis_smoke_test.h"

typedef struct
{
    float vy;
    float vx;
    float vw;
} SpeedCommand;

static SpeedCommand captured_commands[16];
static size_t captured_command_count;
static uint32_t captured_delays[16];
static size_t captured_delay_count;

void Motor_setspeed(float vy, float vx, float vw)
{
    captured_commands[captured_command_count++] = (SpeedCommand){vy, vx, vw};
}

void Motor_Stop(void)
{
    Motor_setspeed(0.0f, 0.0f, 0.0f);
}

osStatus_t osDelay(uint32_t ticks)
{
    captured_delays[captured_delay_count++] = ticks;
    return osOK;
}

int main(void)
{
    Chassis_SmokeTest();

    assert(captured_command_count == 11U);
    assert(captured_commands[1].vy == 20.0f);
    assert(captured_commands[3].vy == -20.0f);
    assert(captured_commands[5].vx == 20.0f);
    assert(captured_commands[7].vx == -20.0f);
    assert(captured_commands[9].vw == 20.0f);
    for (size_t i = 0U; i < captured_command_count; i += 2U)
    {
        assert(captured_commands[i].vy == 0.0f);
        assert(captured_commands[i].vx == 0.0f);
        assert(captured_commands[i].vw == 0.0f);
    }
    assert(captured_delay_count == 11U);
    assert(captured_delays[0] == 500U);
    assert(captured_delays[1] == 1000U);
    assert(captured_delays[2] == 500U);
    puts("chassis smoke test sequence passed");
    return 0;
}
