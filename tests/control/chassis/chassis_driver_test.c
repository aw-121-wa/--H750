#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "control/chassis/chassis.h"
#include "devices/motor/zdt_x42s/zdt_x42s.h"

FDCAN_HandleTypeDef hfdcan1;

typedef struct
{
    uint8_t motor_id;
    int16_t rpm_x10;
    uint16_t acceleration_rpm_s;
    bool synchronized;
} CapturedSpeed;

static CapturedSpeed captured_speed[4];
static size_t captured_speed_count;
static size_t sync_count;
static bool fake_capacity = true;
static size_t fail_speed_call;
static size_t can_init_count;

static void reset_fake(void)
{
    memset(captured_speed, 0, sizeof(captured_speed));
    captured_speed_count = 0U;
    sync_count = 0U;
    fake_capacity = true;
    fail_speed_call = 0U;
    can_init_count = 0U;
}

HAL_StatusTypeDef ZdtX42s_Init(FDCAN_HandleTypeDef *hfdcan)
{
    assert(hfdcan == &hfdcan1);
    can_init_count++;
    return HAL_OK;
}

bool ZdtX42s_HasTxCapacity(uint32_t frame_count)
{
    assert(frame_count == 5U);
    return fake_capacity;
}

HAL_StatusTypeDef ZdtX42s_SetSpeedX10(uint8_t motor_id,
                                      int16_t rpm_x10,
                                      uint16_t acceleration_rpm_s,
                                      bool synchronized)
{
    const size_t call_number = captured_speed_count + 1U;

    if (fail_speed_call == call_number)
    {
        return HAL_ERROR;
    }
    captured_speed[captured_speed_count++] = (CapturedSpeed){
        motor_id, rpm_x10, acceleration_rpm_s, synchronized
    };
    return HAL_OK;
}

HAL_StatusTypeDef ZdtX42s_Sync(void)
{
    sync_count++;
    return HAL_OK;
}

static void test_init_starts_can_and_sends_zero_speed(void)
{
    reset_fake();
    assert(Chassis_Init() == HAL_OK);
    assert(can_init_count == 1U);
    assert(captured_speed_count == 4U);
    assert(sync_count == 1U);
    for (size_t i = 0U; i < 4U; ++i)
    {
        assert(captured_speed[i].motor_id == i + 1U);
        assert(captured_speed[i].rpm_x10 == 0);
        assert(captured_speed[i].acceleration_rpm_s == 1000U);
        assert(captured_speed[i].synchronized);
    }
}

static void test_chassis_set_speed_maps_forward_left_and_rotation(void)
{
    reset_fake();
    assert(Chassis_SetSpeed(20.0f, 0.0f, 0.0f, false) == HAL_OK);
    assert(captured_speed[0].rpm_x10 == 200);
    assert(captured_speed[1].rpm_x10 == 200);
    assert(captured_speed[2].rpm_x10 == -200);
    assert(captured_speed[3].rpm_x10 == -200);

    reset_fake();
    assert(Chassis_SetSpeed(0.0f, 20.0f, 0.0f, false) == HAL_OK);
    assert(captured_speed[0].rpm_x10 == -200);
    assert(captured_speed[1].rpm_x10 == 200);
    assert(captured_speed[2].rpm_x10 == 200);
    assert(captured_speed[3].rpm_x10 == -200);

    reset_fake();
    assert(Chassis_SetSpeed(0.0f, 0.0f, 20.0f, false) == HAL_OK);
    for (size_t i = 0U; i < 4U; ++i)
    {
        assert(captured_speed[i].rpm_x10 == 200);
    }
}

static void test_fifo_shortage_sends_nothing(void)
{
    const int16_t speed[4] = {100, 100, -100, -100};

    reset_fake();
    fake_capacity = false;
    assert(Chassis_SetWheelSpeedX10(speed) == HAL_BUSY);
    assert(captured_speed_count == 0U);
    assert(sync_count == 0U);
}

static void test_partial_failure_does_not_trigger_sync(void)
{
    const int16_t speed[4] = {100, 100, -100, -100};

    reset_fake();
    fail_speed_call = 3U;
    assert(Chassis_SetWheelSpeedX10(speed) == HAL_ERROR);
    assert(captured_speed_count == 2U);
    assert(sync_count == 0U);
}

static void test_non_finite_speed_is_rejected_before_can_enqueue(void)
{
    reset_fake();
    assert(Chassis_SetSpeed(NAN, 0.0f, 0.0f, true) == HAL_ERROR);
    assert(captured_speed_count == 0U);

    reset_fake();
    assert(Chassis_SetSpeed(0.0f, INFINITY, 0.0f, true) == HAL_ERROR);
    assert(captured_speed_count == 0U);

    reset_fake();
    assert(Chassis_SetSpeed(0.0f, 0.0f, -INFINITY, true) == HAL_ERROR);
    assert(captured_speed_count == 0U);
}

int main(void)
{
    test_init_starts_can_and_sends_zero_speed();
    test_chassis_set_speed_maps_forward_left_and_rotation();
    test_fifo_shortage_sends_nothing();
    test_partial_failure_does_not_trigger_sync();
    test_non_finite_speed_is_rejected_before_can_enqueue();
    puts("chassis driver tests passed");
    return 0;
}
