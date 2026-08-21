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

typedef struct
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
} QueuedRxFrame;

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
static FDCAN_HandleTypeDef *active_handle;
static bool position_read_hook_enabled;
static QueuedRxFrame rx_queue[8];
static size_t rx_queue_count;
static size_t rx_queue_index;
static bool single_rx_pending;
static FDCAN_ProtocolStatusTypeDef fake_protocol_status;
static HAL_StatusTypeDef fake_stop_status;
static HAL_StatusTypeDef fake_start_status;
static uint32_t stop_call_count;
static uint32_t start_call_count;

static void trigger_rx_callback(FDCAN_HandleTypeDef *handle);

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

void ZdtX42s_TestPositionReadHook(void)
{
    if (position_read_hook_enabled)
    {
        position_read_hook_enabled = false;
        trigger_rx_callback(active_handle);
    }
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
    active_handle = NULL;
    position_read_hook_enabled = false;
    memset(rx_queue, 0, sizeof(rx_queue));
    rx_queue_count = 0U;
    rx_queue_index = 0U;
    single_rx_pending = false;
    memset(&fake_protocol_status, 0, sizeof(fake_protocol_status));
    fake_stop_status = HAL_OK;
    fake_start_status = HAL_OK;
    stop_call_count = 0U;
    start_call_count = 0U;
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
    start_call_count++;
    return fake_start_status;
}

HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    stop_call_count++;
    return fake_stop_status;
}

HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(
    const FDCAN_HandleTypeDef *hfdcan,
    FDCAN_ProtocolStatusTypeDef *status)
{
    (void)hfdcan;
    *status = fake_protocol_status;
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
    if (rx_queue_index < rx_queue_count)
    {
        *header = rx_queue[rx_queue_index].header;
        memcpy(data, rx_queue[rx_queue_index].data, sizeof(fake_rx_data));
        rx_queue_index++;
    }
    else
    {
        assert(single_rx_pending);
        *header = fake_rx_header;
        memcpy(data, fake_rx_data, sizeof(fake_rx_data));
        single_rx_pending = false;
    }
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t fifo)
{
    (void)hfdcan;
    assert(fifo == FDCAN_RX_FIFO0);
    if (rx_queue_index < rx_queue_count)
    {
        return (uint32_t)(rx_queue_count - rx_queue_index);
    }
    return single_rx_pending ? 1U : 0U;
}

static void trigger_rx_callback(FDCAN_HandleTypeDef *handle)
{
    single_rx_pending = true;
    HAL_FDCAN_RxFifo0Callback(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}

static void enqueue_rx_frame(const FDCAN_RxHeaderTypeDef *header,
                             const uint8_t data[8])
{
    assert(rx_queue_count < (sizeof(rx_queue) / sizeof(rx_queue[0])));
    rx_queue[rx_queue_count].header = *header;
    memcpy(rx_queue[rx_queue_count].data,
           data,
           sizeof(rx_queue[rx_queue_count].data));
    rx_queue_count++;
}

static void test_init_accepts_extended_frames_and_rejects_others(void)
{
    FDCAN_HandleTypeDef handle = {0};

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    assert(captured_filter.IdType == FDCAN_EXTENDED_ID);
    assert(captured_filter.FilterType == FDCAN_FILTER_RANGE);
    assert(captured_filter.FilterID1 == 0x100U);
    assert(captured_filter.FilterID2 == 0x4FFU);
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
    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_3;
    fake_rx_data[0] = 0xF6U;
    fake_rx_data[1] = 0x02U;
    fake_rx_data[2] = 0x6BU;

    fake_tick = 55U;
    trigger_rx_callback(&handle);
    diagnostics = ZdtX42s_GetDiagnostics();
    assert(diagnostics->rx_frame_count == 1U);
    assert(diagnostics->last_rx_id == 0x200U);
    assert(diagnostics->last_function == 0xF6U);
    assert(diagnostics->last_status == 0x02U);
    assert(diagnostics->motor_reply_count[1] == 1U);
    assert(diagnostics->last_motor_reply_ms[1] == 55U);
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
    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_7;
    fake_rx_data[0] = 0x36U;
    fake_rx_data[1] = 0x01U;
    fake_rx_data[2] = 0x00U;
    fake_rx_data[3] = 0x00U;
    fake_rx_data[4] = 0x1CU;
    fake_rx_data[5] = 0x19U;
    fake_rx_data[6] = 0x6BU;
    trigger_rx_callback(&handle);

    fake_tick = 150U;
    assert(ZdtX42s_GetPosition(4U, &position, &age));
    assert(position == -7193);
    assert(age == 25U);
    assert(ZdtX42s_GetDiagnostics()->last_position_reply_ms[3] == 125U);
}

static void test_position_sample_sequence_advances_only_for_valid_reply(void)
{
    FDCAN_HandleTypeDef handle = {0};
    ZdtX42sPositionSample sample = {0};

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);

    fake_tick = 10U;
    fake_rx_header.Identifier = 0x100U;
    fake_rx_header.IdType = FDCAN_EXTENDED_ID;
    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_7;
    fake_rx_data[0] = 0x36U;
    fake_rx_data[1] = 0x00U;
    fake_rx_data[2] = 0x00U;
    fake_rx_data[3] = 0x00U;
    fake_rx_data[4] = 0x00U;
    fake_rx_data[5] = 0x2AU;
    fake_rx_data[6] = 0x6BU;
    trigger_rx_callback(&handle);

    assert(ZdtX42s_GetPositionSample(1U, &sample));
    assert(sample.position_x10_deg == 42);
    assert(sample.timestamp_ms == 10U);
    assert(sample.sequence == 1U);

    fake_tick = 11U;
    fake_rx_data[6] = 0x00U;
    trigger_rx_callback(&handle);
    assert(ZdtX42s_GetPositionSample(1U, &sample));
    assert(sample.sequence == 1U);
    assert(sample.timestamp_ms == 10U);
}

static void test_position_sample_retries_after_interrupted_read(void)
{
    FDCAN_HandleTypeDef handle = {0};
    ZdtX42sPositionSample sample = {0};

    reset_fake();
    active_handle = &handle;
    assert(ZdtX42s_Init(&handle) == HAL_OK);

    fake_tick = 10U;
    fake_rx_header.Identifier = 0x100U;
    fake_rx_header.IdType = FDCAN_EXTENDED_ID;
    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_7;
    fake_rx_data[0] = 0x36U;
    fake_rx_data[1] = 0x00U;
    fake_rx_data[2] = 0x00U;
    fake_rx_data[3] = 0x00U;
    fake_rx_data[4] = 0x00U;
    fake_rx_data[5] = 0x2AU;
    fake_rx_data[6] = 0x6BU;
    trigger_rx_callback(&handle);

    fake_tick = 11U;
    fake_rx_data[5] = 0x63U;
    position_read_hook_enabled = true;
    assert(ZdtX42s_GetPositionSample(1U, &sample));
    assert(sample.position_x10_deg == 99);
    assert(sample.timestamp_ms == 11U);
    assert(sample.sequence == 2U);
}

static void trigger_queued_rx_callback(FDCAN_HandleTypeDef *handle)
{
    HAL_FDCAN_RxFifo0Callback(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}

static void test_rx_callback_drains_all_queued_frames(void)
{
    FDCAN_HandleTypeDef handle = {0};
    FDCAN_RxHeaderTypeDef header = {0};
    uint8_t first_data[8] = {0x36U, 0x00U, 0x00U, 0x00U,
                             0x00U, 0x2AU, 0x6BU, 0x00U};
    uint8_t second_data[8] = {0x36U, 0x00U, 0x00U, 0x00U,
                              0x00U, 0x2BU, 0x6BU, 0x00U};
    const volatile ZdtX42sDiagnostics *diagnostics;

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    header.IdType = FDCAN_EXTENDED_ID;
    header.RxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_7;
    header.Identifier = 0x100U;
    enqueue_rx_frame(&header, first_data);
    header.Identifier = 0x200U;
    enqueue_rx_frame(&header, second_data);

    trigger_queued_rx_callback(&handle);
    diagnostics = ZdtX42s_GetDiagnostics();
    assert(diagnostics->rx_frame_count == 2U);
    assert(diagnostics->motor_reply_count[0] == 1U);
    assert(diagnostics->motor_reply_count[1] == 1U);
}

static void test_invalid_frames_do_not_count_as_motor_replies(void)
{
    FDCAN_HandleTypeDef handle = {0};
    const volatile ZdtX42sDiagnostics *diagnostics;

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    fake_rx_header.Identifier = 0x100U;
    fake_rx_header.IdType = FDCAN_EXTENDED_ID;
    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_7;
    fake_rx_data[0] = 0x36U;
    fake_rx_data[6] = 0x00U;
    trigger_rx_callback(&handle);

    fake_rx_header.RxFrameType = FDCAN_REMOTE_FRAME;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_3;
    fake_rx_data[0] = 0xF6U;
    fake_rx_data[2] = 0x6BU;
    trigger_rx_callback(&handle);

    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.Identifier = 0x101U;
    trigger_rx_callback(&handle);

    fake_rx_header.Identifier = 0x100U;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_3;
    fake_rx_data[0] = 0x36U;
    trigger_rx_callback(&handle);

    fake_rx_header.DataLength = FDCAN_DLC_BYTES_7;
    fake_rx_data[0] = 0xAAU;
    trigger_rx_callback(&handle);

    HAL_FDCAN_RxFifo0Callback(&handle,
                              FDCAN_IT_RX_FIFO0_FULL |
                                  FDCAN_IT_RX_FIFO0_MESSAGE_LOST);

    diagnostics = ZdtX42s_GetDiagnostics();
    assert(diagnostics->motor_reply_count[0] == 0U);
    assert(diagnostics->invalid_tail_count == 1U);
    assert(diagnostics->invalid_dlc_count == 1U);
    assert(diagnostics->invalid_frame_count == 2U);
    assert(diagnostics->unknown_function_count == 1U);
    assert(diagnostics->rx_fifo_full_count == 1U);
    assert(diagnostics->rx_fifo_lost_count == 1U);
}

static void publish_valid_motor_reply(FDCAN_HandleTypeDef *handle,
                                      uint8_t motor_id,
                                      uint32_t tick)
{
    fake_tick = tick;
    fake_rx_header.Identifier = (uint32_t)motor_id << 8U;
    fake_rx_header.IdType = FDCAN_EXTENDED_ID;
    fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_3;
    fake_rx_data[0] = 0xF6U;
    fake_rx_data[1] = 0x00U;
    fake_rx_data[2] = 0x6BU;
    trigger_rx_callback(handle);
}

static void test_motor_online_mask_uses_reply_freshness(void)
{
    FDCAN_HandleTypeDef handle = {0};

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    for (uint8_t motor_id = 1U; motor_id <= ZDT_X42S_MOTOR_COUNT; ++motor_id)
    {
        publish_valid_motor_reply(&handle, motor_id, 100U + motor_id);
    }
    assert(ZdtX42s_GetMotorOnlineMask(302U) == 0x0EU);
    assert(ZdtX42s_GetMotorOnlineMask(300U) == 0x0FU);
}

static void test_can_error_state_is_current_and_counted(void)
{
    FDCAN_HandleTypeDef handle = {0};
    const volatile ZdtX42sDiagnostics *diagnostics;
    ZdtX42sHealth health;

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);

    fake_tick = 10U;
    HAL_FDCAN_ErrorStatusCallback(&handle, FDCAN_IT_ERROR_WARNING);
    diagnostics = ZdtX42s_GetDiagnostics();
    health = ZdtX42s_GetHealth(10U);
    assert(diagnostics->bus_error_count == 1U);
    assert(diagnostics->error_warning_count == 1U);
    assert(diagnostics->last_error_ms == 10U);
    assert(health.bus_state == ZDT_CAN_WARNING);

    fake_tick = 20U;
    HAL_FDCAN_ErrorStatusCallback(&handle, FDCAN_IT_ERROR_PASSIVE);
    assert(ZdtX42s_GetHealth(20U).bus_state == ZDT_CAN_ERROR_PASSIVE);

    fake_tick = 30U;
    HAL_FDCAN_ErrorStatusCallback(&handle, FDCAN_IT_BUS_OFF);
    diagnostics = ZdtX42s_GetDiagnostics();
    assert(diagnostics->error_passive_count == 1U);
    assert(diagnostics->bus_off_count == 1U);
    assert(ZdtX42s_GetHealth(30U).bus_state == ZDT_CAN_BUS_OFF);
}

static void test_bus_off_recovery_sends_stop_before_ready(void)
{
    FDCAN_HandleTypeDef handle = {0};
    uint32_t initial_start_count;
    ZdtX42sHealth health;

    reset_fake();
    assert(ZdtX42s_Init(&handle) == HAL_OK);
    initial_start_count = start_call_count;
    HAL_FDCAN_ErrorStatusCallback(&handle, FDCAN_IT_BUS_OFF);

    ZdtX42s_Service(10U);
    assert(stop_call_count == 1U);
    assert(start_call_count == initial_start_count + 1U);
    assert(captured_frame_count == 5U);
    for (size_t i = 0U; i < 4U; ++i)
    {
        assert(captured_frames[i].data[0] == 0xF6U);
        assert(captured_frames[i].data[4] == 0x00U);
        assert(captured_frames[i].data[5] == 0x00U);
    }

    for (uint8_t motor_id = 1U; motor_id <= ZDT_X42S_MOTOR_COUNT; ++motor_id)
    {
        publish_valid_motor_reply(&handle, motor_id, 20U + motor_id);
    }
    fake_protocol_status.BusOff = 0U;
    ZdtX42s_Service(30U);
    health = ZdtX42s_GetHealth(30U);
    assert(health.bus_state == ZDT_CAN_OK);
    assert(health.motor_online_mask == 0x0FU);
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
    test_position_sample_sequence_advances_only_for_valid_reply();
    test_position_sample_retries_after_interrupted_read();
    test_rx_callback_drains_all_queued_frames();
    test_invalid_frames_do_not_count_as_motor_replies();
    test_motor_online_mask_uses_reply_freshness();
    test_can_error_state_is_current_and_counted();
    test_bus_off_recovery_sends_stop_before_ready();
    puts("ZDT CAN driver tests passed");
    return 0;
}
