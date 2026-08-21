#include "control/localization/localization_service.h"

#include <string.h>

#include "control/localization/localization.h"
#include "devices/motor/zdt_x42s/zdt_x42s.h"

#define POSITION_STALE_MS 200U
#define POSITION_MAX_SAMPLE_SPAN_MS 30U
#define POSITION_MAX_DELTA_X10_DEG 6000
#define POSITION_COMPLETE_MASK ((1U << ZDT_X42S_MOTOR_COUNT) - 1U)

static int32_t s_previous_positions[ZDT_X42S_MOTOR_COUNT];
static uint32_t s_start_sequence[ZDT_X42S_MOTOR_COUNT];
static uint8_t s_sample_mask;
static bool s_sample_pending;
static bool s_have_previous;
static bool s_synchronized_sample_valid;
static bool s_unreasonable_jump;

static void StartSample(void)
{
    ZdtX42sPositionSample sample;
    bool request_ok = true;

    if (!ZdtX42s_HasTxCapacity(ZDT_X42S_MOTOR_COUNT))
    {
        return;
    }
    for (uint8_t motor_id = 1U; motor_id <= ZDT_X42S_MOTOR_COUNT; ++motor_id)
    {
        s_start_sequence[motor_id - 1U] =
            ZdtX42s_GetPositionSample(motor_id, &sample) ? sample.sequence : 0U;
    }
    for (uint8_t motor_id = 1U; motor_id <= ZDT_X42S_MOTOR_COUNT; ++motor_id)
    {
        if (ZdtX42s_RequestPosition(motor_id) != HAL_OK)
        {
            request_ok = false;
        }
    }
    if (request_ok)
    {
        s_sample_mask = 0U;
        s_sample_pending = true;
    }
}

static void FinishSample(const ZdtX42sPositionSample samples[ZDT_X42S_MOTOR_COUNT],
                         float imu_yaw_deg)
{
    int32_t positions[ZDT_X42S_MOTOR_COUNT];
    uint32_t earliest = samples[0].timestamp_ms;
    uint32_t latest = samples[0].timestamp_ms;

    s_synchronized_sample_valid = false;
    for (uint8_t i = 0U; i < ZDT_X42S_MOTOR_COUNT; ++i)
    {
        positions[i] = samples[i].position_x10_deg;
        if ((int32_t)(samples[i].timestamp_ms - earliest) < 0)
        {
            earliest = samples[i].timestamp_ms;
        }
        if ((int32_t)(samples[i].timestamp_ms - latest) > 0)
        {
            latest = samples[i].timestamp_ms;
        }
    }
    if ((latest - earliest) > POSITION_MAX_SAMPLE_SPAN_MS)
    {
        return;
    }
    if (s_have_previous)
    {
        for (uint8_t i = 0U; i < ZDT_X42S_MOTOR_COUNT; ++i)
        {
            int64_t delta = (int64_t)positions[i] - s_previous_positions[i];
            if (delta > POSITION_MAX_DELTA_X10_DEG ||
                delta < -POSITION_MAX_DELTA_X10_DEG)
            {
                s_unreasonable_jump = true;
                return;
            }
        }
    }
    memcpy(s_previous_positions, positions, sizeof(positions));
    s_have_previous = true;
    s_synchronized_sample_valid = true;
    Localization_Update(positions, imu_yaw_deg, earliest + (latest - earliest) / 2U);
}

void LocalizationService_Init(void)
{
    memset(s_previous_positions, 0, sizeof(s_previous_positions));
    memset(s_start_sequence, 0, sizeof(s_start_sequence));
    s_sample_mask = 0U;
    s_sample_pending = false;
    s_have_previous = false;
    s_synchronized_sample_valid = false;
    s_unreasonable_jump = false;
    Localization_Init();
}

void LocalizationService_Tick(uint32_t now_ms, float imu_yaw_deg)
{
    ZdtX42sPositionSample samples[ZDT_X42S_MOTOR_COUNT] = {0};

    (void)now_ms;
    if (s_sample_pending)
    {
        for (uint8_t motor_id = 1U; motor_id <= ZDT_X42S_MOTOR_COUNT; ++motor_id)
        {
            if (ZdtX42s_GetPositionSample(motor_id, &samples[motor_id - 1U]) &&
                samples[motor_id - 1U].sequence != s_start_sequence[motor_id - 1U])
            {
                s_sample_mask |= (uint8_t)(1U << (motor_id - 1U));
            }
        }
        if (s_sample_mask == POSITION_COMPLETE_MASK)
        {
            FinishSample(samples, imu_yaw_deg);
            s_sample_pending = false;
        }
    }
    if (!s_sample_pending)
    {
        StartSample();
    }
}

LocalizationServiceStatus LocalizationService_GetStatus(uint32_t now_ms)
{
    LocalizationServiceStatus status = {0};

    status.synchronized_sample_valid = s_synchronized_sample_valid;
    status.unreasonable_jump = s_unreasonable_jump;
    for (uint8_t motor_id = 1U; motor_id <= ZDT_X42S_MOTOR_COUNT; ++motor_id)
    {
        ZdtX42sPositionSample sample;

        if (ZdtX42s_GetPositionSample(motor_id, &sample) &&
            (now_ms - sample.timestamp_ms) <= POSITION_STALE_MS)
        {
            status.motor_valid_mask |= (uint8_t)(1U << (motor_id - 1U));
            if ((now_ms - sample.timestamp_ms) > status.oldest_sample_age_ms)
            {
                status.oldest_sample_age_ms = now_ms - sample.timestamp_ms;
            }
        }
    }
    return status;
}
