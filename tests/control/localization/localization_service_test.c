#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "control/localization/localization_service.h"
#include "devices/motor/zdt_x42s/zdt_x42s.h"

static ZdtX42sPositionSample samples[ZDT_X42S_MOTOR_COUNT];
static uint32_t request_count;
static uint32_t update_count;
static int32_t last_positions[ZDT_X42S_MOTOR_COUNT];
static uint32_t last_update_time;

void Localization_Init(void)
{
}

void Localization_Update(const int32_t wheel_position_x10_deg[4],
                         float imu_yaw_deg,
                         uint32_t now_ms)
{
    (void)imu_yaw_deg;
    memcpy(last_positions, wheel_position_x10_deg, sizeof(last_positions));
    last_update_time = now_ms;
    update_count++;
}

HAL_StatusTypeDef ZdtX42s_RequestPosition(uint8_t motor_id)
{
    assert(motor_id >= 1U && motor_id <= ZDT_X42S_MOTOR_COUNT);
    request_count++;
    return HAL_OK;
}

bool ZdtX42s_HasTxCapacity(uint32_t frame_count)
{
    return frame_count == ZDT_X42S_MOTOR_COUNT;
}

bool ZdtX42s_GetPositionSample(uint8_t motor_id,
                                ZdtX42sPositionSample *sample)
{
    assert(motor_id >= 1U && motor_id <= ZDT_X42S_MOTOR_COUNT);
    *sample = samples[motor_id - 1U];
    return sample->valid;
}

bool ZdtX42s_GetPosition(uint8_t motor_id,
                          int32_t *position_x10_deg,
                          uint32_t *age_ms)
{
    (void)motor_id;
    (void)position_x10_deg;
    (void)age_ms;
    return false;
}

static void reset_fake(void)
{
    memset(samples, 0, sizeof(samples));
    request_count = 0U;
    update_count = 0U;
    memset(last_positions, 0, sizeof(last_positions));
    last_update_time = 0U;
}

static void publish_all(uint32_t sequence, uint32_t first_timestamp)
{
    for (uint8_t i = 0U; i < ZDT_X42S_MOTOR_COUNT; ++i)
    {
        samples[i].position_x10_deg = (int32_t)(100 + i);
        samples[i].timestamp_ms = first_timestamp + i;
        samples[i].sequence = sequence;
        samples[i].valid = true;
    }
}

static void test_batch_request_and_complete_reply_update_once(void)
{
    reset_fake();
    LocalizationService_Init();

    LocalizationService_Tick(0U, 12.0f);
    assert(request_count == 4U);
    assert(update_count == 0U);

    publish_all(1U, 1U);
    LocalizationService_Tick(5U, 12.0f);
    assert(update_count == 1U);
    assert(last_positions[0] == 100);
    assert(last_positions[3] == 103);
    assert(last_update_time == 2U);
    assert(request_count == 8U);
}

static void test_old_and_incomplete_replies_never_form_a_snapshot(void)
{
    reset_fake();
    publish_all(1U, 0U);
    LocalizationService_Init();

    LocalizationService_Tick(0U, 0.0f);
    LocalizationService_Tick(5U, 0.0f);
    assert(update_count == 0U);

    for (uint8_t i = 0U; i < 3U; ++i)
    {
        samples[i].sequence = 2U;
        samples[i].timestamp_ms = 6U;
    }
    LocalizationService_Tick(10U, 0.0f);
    assert(update_count == 0U);
}

int main(void)
{
    test_batch_request_and_complete_reply_update_once();
    test_old_and_incomplete_replies_never_form_a_snapshot();
    puts("Localization service tests passed");
    return 0;
}
