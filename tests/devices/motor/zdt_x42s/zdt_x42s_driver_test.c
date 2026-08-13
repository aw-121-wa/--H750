#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "devices/motor/zdt_x42s/zdt_x42s.h"

typedef struct
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CapturedFrame;

static FDCAN_FilterTypeDef captured_filter;
static uint32_t captured_global_filter[4];
static uint32_t captured_notifications;
static CapturedFrame captured_frames[8];
static size_t captured_frame_count;
static uint32_t fake_free_level = 8U;
static HAL_StatusTypeDef fake_tx_status = HAL_OK;
static HAL_StatusTypeDef fake_filter_status = HAL_OK;
static FDCAN_RxHeaderTypeDef fake_rx_header;
static uint8_t fake_rx_data[8];
static uint32_t fake_tick;

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

static void reset_fake(void)
{
    memset(&captured_filter, 0, sizeof(captured_filter));
    memset(captured_global_filter, 0, sizeof(captured_global_filter));
    memset(captured_frames, 0, sizeof(captured_frames));
    captured_notifications = 0U;
    captured_frame_count = 0U;
    fake_free_level = 8U;
    fake_tx_status = HAL_OK;
    fake_filter_status = HAL_OK;
    memset(&fake_rx_header, 0, sizeof(fake_rx_header));
    memset(fake_rx_data, 0, sizeof(fake_rx_data));
    fake_tick = 0U;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan,
                                         FDCAN_FilterTypeDef *filter)
{
    (void)hfdcan;
    captured_filter = *filter;
    return fake_filter_status;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
                                               uint32_t non_matching_std,
                                               uint32_t non_matching_ext,
                                               uint32_t reject_remote_std,
                                               uint32_t reject_remote_ext)
{
    (void)hfdcan;
    captured_global_filter[0] = non_matching_std;
    captured_global_filter[1] = non_matching_ext;
    captured_global_filter[2] = reject_remote_std;
    captured_global_filter[3] = reject_remote_ext;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
                                                 uint32_t notifications,
                                                 uint32_t buffer_indexes)
{
    (void)hfdcan;
    (void)buffer_indexes;
    captured_notifications = notifications;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                FDCAN_TxHeaderTypeDef *header,
                                                uint8_t *data)
{
    (void)hfdcan;
    if (fake_tx_status == HAL_OK && captured_frame_count < 8U)
    {
        CapturedFrame *frame = &captured_frames[captured_frame_count++];
        frame->header = *header;
        memcpy(frame->data, data, sizeof(frame->data));
    }
    return fake_tx_status;
}

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return fake_free_level;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t fifo,
                                        FDCAN_RxHeaderTypeDef *header,
                                        uint8_t *data)
{
    (void)hfdcan;
    assert(fifo == FDCAN_RX_FIFO0);
    *header = fake_rx_header;
    memcpy(data, fake_rx_data, sizeof(fake_rx_data));
    return HAL_OK;
}

static void test_init_accepts_extended_frames_and_rejects_others(void)
{
    FDCAN_HandleTypeDef handle = {0};

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    assert(captured_filter.IdType == FDCAN_EXTENDED_ID);
    assert(captured_filter.FilterType == FDCAN_FILTER_MASK);
    assert(captured_filter.FilterID1 == 0U && captured_filter.FilterID2 == 0U);
    assert(captured_global_filter[0] == FDCAN_REJECT);
    assert(captured_global_filter[1] == FDCAN_REJECT);
    assert(captured_global_filter[2] == FDCAN_REJECT_REMOTE);
    assert(captured_global_filter[3] == FDCAN_REJECT_REMOTE);
    assert((captured_notifications & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U);
}

static void test_speed_command_uses_extended_id_and_x_firmware_layout(void)
{
    FDCAN_HandleTypeDef handle = {0};
    const uint8_t expected[] = {
        0xF6, 0x01, 0x03, 0xE8, 0x04, 0xD2, 0x01, 0x6B
    };

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    assert(ZdtX42s_SetSpeedX10(3U, -1234, 1000U, true) == HAL_OK);
    assert(captured_frame_count == 1U);
    assert(captured_frames[0].header.Identifier == 0x300U);
    assert(captured_frames[0].header.IdType == FDCAN_EXTENDED_ID);
    assert(captured_frames[0].header.DataLength == FDCAN_DLC_BYTES_8);
    assert(memcmp(captured_frames[0].data, expected, sizeof(expected)) == 0);
}

static void test_invalid_motor_id_and_fifo_capacity(void)
{
    FDCAN_HandleTypeDef handle = {0};

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    assert(ZdtX42s_SetSpeedX10(0U, 100, 0U, true) == HAL_ERROR);
    assert(captured_frame_count == 0U);
    fake_free_level = 4U;
    assert(!ZdtX42s_HasTxCapacity(5U));
    fake_free_level = 5U;
    assert(ZdtX42s_HasTxCapacity(5U));
}

static void test_failed_init_does_not_leave_driver_ready(void)
{
    FDCAN_HandleTypeDef handle = {0};

    reset_fake();
    fake_filter_status = HAL_ERROR;
    assert(ZdtX42s_Init(&handle) == HAL_ERROR);
    assert(!ZdtX42s_HasTxCapacity(1U));
}

static void test_sync_command_and_diagnostics(void)
{
    FDCAN_HandleTypeDef handle = {0};
    const volatile ZdtX42sDiagnostics *diagnostics;

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    assert(ZdtX42s_Sync() == HAL_OK);
    assert(captured_frames[0].header.Identifier == 0U);
    assert(captured_frames[0].header.DataLength == FDCAN_DLC_BYTES_3);
    diagnostics = ZdtX42s_GetDiagnostics();
    assert(diagnostics->tx_success_count == 1U);

    fake_tx_status = HAL_ERROR;
    assert(ZdtX42s_Sync() == HAL_ERROR);
    assert(diagnostics->tx_failure_count == 1U);
}

static void test_rx_callback_records_motor_reply(void)
{
    FDCAN_HandleTypeDef handle = {0};
    const volatile ZdtX42sDiagnostics *diagnostics;

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    fake_rx_header.Identifier = 0x200U;
    fake_rx_header.IdType = FDCAN_EXTENDED_ID;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_3;
    fake_rx_data[0] = 0xF6U;
    fake_rx_data[1] = 0x02U;
    fake_rx_data[2] = 0x6BU;

    HAL_FDCAN_RxFifo0Callback(&handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    diagnostics = ZdtX42s_GetDiagnostics();
    assert(diagnostics->rx_frame_count == 1U);
    assert(diagnostics->last_rx_id == 0x200U);
    assert(diagnostics->last_function == 0xF6U);
    assert(diagnostics->last_status == 0x02U);
    assert(diagnostics->motor_reply_count[1] == 1U);
}

static void test_position_request_and_reply(void)
{
    FDCAN_HandleTypeDef handle = {0};
    int32_t position = 0;
    uint32_t age = 0U;

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    assert(ZdtX42s_RequestPosition(4U) == HAL_OK);
    assert(captured_frames[0].header.Identifier == 0x400U);
    assert(captured_frames[0].header.DataLength == FDCAN_DLC_BYTES_2);
    assert(captured_frames[0].data[0] == 0x36U);
    assert(captured_frames[0].data[1] == 0x6BU);

    fake_tick = 125U;
    fake_rx_header.Identifier = 0x400U;
    fake_rx_header.IdType = FDCAN_EXTENDED_ID;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_7;
    fake_rx_data[0] = 0x36U;
    fake_rx_data[1] = 0x01U;
    fake_rx_data[2] = 0x00U;
    fake_rx_data[3] = 0x00U;
    fake_rx_data[4] = 0x1CU;
    fake_rx_data[5] = 0x19U;
    fake_rx_data[6] = 0x6BU;
    HAL_FDCAN_RxFifo0Callback(&handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

    fake_tick = 150U;
    assert(ZdtX42s_GetPosition(4U, &position, &age));
    assert(position == -7193);
    assert(age == 25U);
}

int main(void)
{
    test_init_accepts_extended_frames_and_rejects_others();
    test_speed_command_uses_extended_id_and_x_firmware_layout();
    test_invalid_motor_id_and_fifo_capacity();
    test_failed_init_does_not_leave_driver_ready();
    test_sync_command_and_diagnostics();
    test_rx_callback_records_motor_reply();
    test_position_request_and_reply();
    puts("ZDT CAN driver tests passed");
    return 0;
}
