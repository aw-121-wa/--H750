#include "control/localization/localization_service.h"

#include <string.h>

#include "control/localization/localization.h"
#include "devices/motor/zdt_x42s/zdt_x42s.h"

#define POSITION_REPLY_TIMEOUT_MS 8U
#define POSITION_STALE_MS 200U
#define POSITION_MAX_SAMPLE_SPAN_MS 30U
#define POSITION_MAX_DELTA_X10_DEG 6000

static int32_t s_positions[ZDT_X42S_MOTOR_COUNT];
static int32_t s_previous_positions[ZDT_X42S_MOTOR_COUNT];
static uint32_t s_sample_times[ZDT_X42S_MOTOR_COUNT];
static uint32_t s_request_time;
static uint8_t s_motor_index;
static uint8_t s_sample_mask;
static bool s_waiting;
static bool s_have_previous;
static bool s_synchronized_sample_valid;
static bool s_unreasonable_jump;

void LocalizationService_Init(void)
{
    memset(s_positions, 0, sizeof(s_positions));
    memset(s_previous_positions, 0, sizeof(s_previous_positions));
    memset(s_sample_times, 0, sizeof(s_sample_times));
    s_request_time = 0U;
    s_motor_index = 0U;
    s_sample_mask = 0U;
    s_waiting = false;
    s_have_previous = false;
    s_synchronized_sample_valid = false;
    s_unreasonable_jump = false;
    Localization_Init();
}

static void FinishSample(float imu_yaw_deg, uint32_t now_ms)
{
    uint32_t earliest = s_sample_times[0];
    uint32_t latest = s_sample_times[0];

    s_synchronized_sample_valid = false;
    if (s_sample_mask != 0x0FU)
    {
        s_sample_mask = 0U;
        return;
    }
    for (uint32_t i = 1U; i < ZDT_X42S_MOTOR_COUNT; ++i)
    {
        if ((int32_t)(s_sample_times[i] - earliest) < 0)
        {
            earliest = s_sample_times[i];
        }
        if ((int32_t)(s_sample_times[i] - latest) > 0)
        {
            latest = s_sample_times[i];
        }
    }
    if ((latest - earliest) > POSITION_MAX_SAMPLE_SPAN_MS)
    {
        s_sample_mask = 0U;
        return;
    }
    if (s_have_previous)
    {
        for (uint32_t i = 0U; i < ZDT_X42S_MOTOR_COUNT; ++i)
        {
            int64_t delta = (int64_t)s_positions[i] -
                            (int64_t)s_previous_positions[i];
            if (delta > POSITION_MAX_DELTA_X10_DEG ||
                delta < -POSITION_MAX_DELTA_X10_DEG)
            {
                s_unreasonable_jump = true;
                s_sample_mask = 0U;
                return;
            }
        }
    }
    memcpy(s_previous_positions, s_positions, sizeof(s_positions));
    s_have_previous = true;
    s_synchronized_sample_valid = true;
    Localization_Update(s_positions, imu_yaw_deg, now_ms);
    s_sample_mask = 0U;
}

void LocalizationService_Tick(uint32_t now_ms, float imu_yaw_deg)
{
    int32_t position;
    uint32_t age_ms;
    uint32_t sample_time;
    bool has_sample;

    if (!s_waiting)
    {
        if (ZdtX42s_RequestPosition(s_motor_index + 1U) == HAL_OK)
        {
            s_request_time = now_ms;
            s_waiting = true;
        }
        return;
    }

    has_sample = ZdtX42s_GetPosition(s_motor_index + 1U, &position, &age_ms);
    if (has_sample)
    {
        sample_time = now_ms - age_ms;
    }
    else
    {
        sample_time = 0U;
    }
    if (has_sample &&
        age_ms <= (now_ms - s_request_time) &&
        sample_time != s_sample_times[s_motor_index])
    {
        s_positions[s_motor_index] = position;
        s_sample_times[s_motor_index] = now_ms - age_ms;
        s_sample_mask |= (uint8_t)(1U << s_motor_index);
        s_waiting = false;
    }
    else if ((now_ms - s_request_time) >= POSITION_REPLY_TIMEOUT_MS)
    {
        s_waiting = false;
    }
    else
    {
        return;
    }

    s_motor_index++;
    if (s_motor_index >= ZDT_X42S_MOTOR_COUNT)
    {
        s_motor_index = 0U;
        FinishSample(imu_yaw_deg, now_ms);
    }
}

LocalizationServiceStatus LocalizationService_GetStatus(uint32_t now_ms)
{
    LocalizationServiceStatus status = {0};

    status.synchronized_sample_valid = s_synchronized_sample_valid;
    status.unreasonable_jump = s_unreasonable_jump;
    for (uint8_t motor_id = 1U; motor_id <= ZDT_X42S_MOTOR_COUNT; ++motor_id)
    {
        int32_t position;
        uint32_t age_ms;

        if (ZdtX42s_GetPosition(motor_id, &position, &age_ms) &&
            age_ms <= POSITION_STALE_MS)
        {
            status.motor_valid_mask |= (uint8_t)(1U << (motor_id - 1U));
            if (age_ms > status.oldest_sample_age_ms)
            {
                status.oldest_sample_age_ms = age_ms;
            }
        }
        (void)position;
    }
    (void)now_ms;
    return status;
}
