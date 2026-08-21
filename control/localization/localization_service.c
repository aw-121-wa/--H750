#include "control/localization/localization_service.h"

#include <string.h>

#include "control/localization/localization.h"
#include "devices/motor/zdt_x42s/zdt_x42s.h"

#define POSITION_REPLY_TIMEOUT_MS 20U
#define POSITION_SAMPLE_PERIOD_MS 20U
#define POSITION_STALE_MS 200U
#define POSITION_MAX_SAMPLE_SPAN_MS 10U
#define POSITION_MAX_DELTA_X10_DEG 6000
#define POSITION_COMPLETE_MASK ((1U << ZDT_X42S_MOTOR_COUNT) - 1U)

static int32_t s_previous_positions[ZDT_X42S_MOTOR_COUNT];
static uint32_t s_start_sequence[ZDT_X42S_MOTOR_COUNT];
static uint8_t s_sample_mask;
static bool s_sample_pending;
static uint32_t s_sample_start_ms;
static uint32_t s_next_sample_ms;
static uint32_t s_sample_timeout_count;
static uint32_t s_incomplete_sample_count;
static uint32_t s_sample_span_reject_count;
static uint8_t s_last_missing_mask;
static bool s_have_previous;
static bool s_synchronized_sample_valid;
static bool s_unreasonable_jump;

static bool TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool StartSample(uint32_t now_ms)
{
    ZdtX42sPositionSample sample;
    bool request_ok = true;

    if (!TimeReached(now_ms, s_next_sample_ms))
    {
        return false;
    }
    if (!ZdtX42s_HasTxCapacity(ZDT_X42S_MOTOR_COUNT))
    {
        return false;
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
    if (!request_ok)
    {
        s_sample_mask = 0U;
        s_sample_pending = false;
        return false;
    }

    s_sample_mask = 0U;
    s_sample_start_ms = now_ms;
    s_sample_pending = true;
    return true;
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
        s_sample_span_reject_count++;
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
    s_sample_start_ms = 0U;
    s_next_sample_ms = 0U;
    s_sample_timeout_count = 0U;
    s_incomplete_sample_count = 0U;
    s_sample_span_reject_count = 0U;
    s_last_missing_mask = 0U;
    s_have_previous = false;
    s_synchronized_sample_valid = false;
    s_unreasonable_jump = false;
    Localization_Init();
}

void LocalizationService_Tick(uint32_t now_ms, float imu_yaw_deg)
{
    ZdtX42sPositionSample samples[ZDT_X42S_MOTOR_COUNT] = {0};

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
            s_sample_mask = 0U;
            s_next_sample_ms = now_ms + POSITION_SAMPLE_PERIOD_MS;
            return;
        }
        if ((now_ms - s_sample_start_ms) >= POSITION_REPLY_TIMEOUT_MS)
        {
            s_last_missing_mask =
                (uint8_t)(POSITION_COMPLETE_MASK & ~s_sample_mask);
            s_sample_timeout_count++;
            s_incomplete_sample_count++;
            s_sample_pending = false;
            s_sample_mask = 0U;
            s_next_sample_ms = now_ms + POSITION_SAMPLE_PERIOD_MS;
            return;
        }
    }
    if (!s_sample_pending && TimeReached(now_ms, s_next_sample_ms))
    {
        (void)StartSample(now_ms);
    }
}

LocalizationServiceStatus LocalizationService_GetStatus(uint32_t now_ms)
{
    LocalizationServiceStatus status = {0};

    status.synchronized_sample_valid = s_synchronized_sample_valid;
    status.unreasonable_jump = s_unreasonable_jump;
    status.last_missing_mask = s_last_missing_mask;
    status.sample_timeout_count = s_sample_timeout_count;
    status.incomplete_sample_count = s_incomplete_sample_count;
    status.sample_span_reject_count = s_sample_span_reject_count;
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
