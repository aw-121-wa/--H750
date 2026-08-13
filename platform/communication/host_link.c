#include "platform/communication/host_link.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "platform/communication/host_dispatcher.h"
#include "platform/communication/host_protocol.h"
#include "usart.h"

#define HOST_RX_BUFFER_SIZE 512U
#define HOST_TX_SLOT_COUNT 8U
#define HOST_COMMAND_QUEUE_DEPTH 8U
#define HOST_POSE_REPORT_PERIOD_MS 100U

typedef struct __attribute__((aligned(32)))
{
    uint8_t data[HOST_PROTOCOL_MAX_FRAME_SIZE];
    uint16_t length;
} HostTxSlot;

typedef struct
{
    NavigationPose pose;
    uint8_t motor_valid_mask;
    uint8_t imu_valid;
    uint16_t fault_flags;
    uint16_t current_waypoint;
    uint8_t navigation_state;
    uint8_t pose_state;
} HostTelemetry;

extern DMA_HandleTypeDef hdma_usart1_rx;

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t s_rx_buffer[HOST_RX_BUFFER_SIZE];

__attribute__((section(".dma_buffer"), aligned(32)))
static HostTxSlot s_tx_slots[HOST_TX_SLOT_COUNT];

static HostProtocolParser s_parser;
static HostDispatcher s_dispatcher;
static osMessageQueueId_t s_command_queue;
static uint16_t s_tx_sequence;
static uint16_t s_rx_read_index;
static volatile uint8_t s_tx_read;
static uint8_t s_tx_write;
static volatile bool s_tx_busy;
static volatile bool s_rx_restart_requested;
static volatile uint32_t s_telemetry_generation;
static HostTelemetry s_telemetry;
static uint32_t s_last_pose_report_ms;

static void WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void WriteI16(uint8_t *data, int16_t value)
{
    WriteU16(data, (uint16_t)value);
}

static void WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static void WriteI32(uint8_t *data, int32_t value)
{
    WriteU32(data, (uint32_t)value);
}

static int16_t ClampI16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)lroundf(value);
}

static bool QueueFrame(HostMessageType type,
                       uint16_t sequence,
                       const uint8_t *payload,
                       uint16_t payload_length)
{
    uint8_t next = (uint8_t)((s_tx_write + 1U) % HOST_TX_SLOT_COUNT);
    HostTxSlot *slot;
    size_t length;

    taskENTER_CRITICAL();
    if (next == s_tx_read)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    slot = &s_tx_slots[s_tx_write];
    length = HostProtocol_Encode(type, sequence, payload, payload_length,
                                 slot->data, sizeof(slot->data));
    if (length == 0U)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    slot->length = (uint16_t)length;
    __DMB();
    s_tx_write = next;
    taskEXIT_CRITICAL();
    return true;
}

static void QueueDispatchReply(const HostDispatchOutput *output)
{
    if (output != NULL && output->reply_ready)
    {
        (void)QueueFrame(output->reply_type, output->reply_sequence,
                         output->reply_payload, sizeof(output->reply_payload));
    }
}

static void OnFrame(const HostFrame *frame, void *context)
{
    HostDispatchOutput output;

    (void)context;
    taskENTER_CRITICAL();
    output = HostDispatcher_Handle(&s_dispatcher, frame, HAL_GetTick());
    taskEXIT_CRITICAL();
    if (output.force_pose_report)
    {
        s_last_pose_report_ms = 0U;
    }
    if (output.command_ready &&
        (s_command_queue == NULL ||
         osMessageQueuePut(s_command_queue, &output.command, 0U, 0U) != osOK))
    {
        output = HostDispatcher_CompleteCommand(&s_dispatcher,
                                                &output.command,
                                                false, 2U);
    }
    QueueDispatchReply(&output);
}

static HAL_StatusTypeDef StartReceive(void)
{
    HAL_StatusTypeDef status;

    s_rx_read_index = 0U;
    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
    SCB_CleanDCache_by_Addr((uint32_t *)s_rx_buffer, sizeof(s_rx_buffer));
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    status = HAL_UART_Receive_DMA(&huart1, s_rx_buffer, sizeof(s_rx_buffer));
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT | DMA_IT_TC);
    }
    return status;
}

HAL_StatusTypeDef HostLink_Init(void)
{
    memset(&s_telemetry, 0, sizeof(s_telemetry));
    memset(s_tx_slots, 0, sizeof(s_tx_slots));
    s_command_queue = osMessageQueueNew(HOST_COMMAND_QUEUE_DEPTH,
                                        sizeof(HostCommand), NULL);
    if (s_command_queue == NULL)
    {
        return HAL_ERROR;
    }
    HostProtocol_Init(&s_parser, OnFrame, NULL);
    s_tx_read = 0U;
    s_tx_write = 0U;
    s_tx_busy = false;
    s_rx_restart_requested = false;
    s_telemetry_generation = 0U;
    s_last_pose_report_ms = 0U;
    HostDispatcher_Init(&s_dispatcher, HAL_GetTick());
    return StartReceive();
}

static void ProcessReceive(void)
{
    uint16_t write_index;

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)s_rx_buffer,
                                    sizeof(s_rx_buffer));
    }
    write_index = (uint16_t)(HOST_RX_BUFFER_SIZE -
                             __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    write_index %= HOST_RX_BUFFER_SIZE;
    while (s_rx_read_index != write_index)
    {
        HostProtocol_Consume(&s_parser, &s_rx_buffer[s_rx_read_index], 1U);
        s_rx_read_index = (uint16_t)((s_rx_read_index + 1U) %
                                     HOST_RX_BUFFER_SIZE);
    }
}

static void StartNextTransmit(void)
{
    HostTxSlot *slot;

    if (s_tx_busy || s_tx_read == s_tx_write)
    {
        return;
    }
    slot = &s_tx_slots[s_tx_read];
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanDCache_by_Addr((uint32_t *)slot,
                                (int32_t)((sizeof(*slot) + 31U) & ~31U));
    }
    if (HAL_UART_Transmit_DMA(&huart1, slot->data, slot->length) == HAL_OK)
    {
        s_tx_busy = true;
    }
}

static HostTelemetry ReadTelemetry(void)
{
    HostTelemetry snapshot;
    uint32_t before;
    uint32_t after;

    do
    {
        before = s_telemetry_generation;
        __DMB();
        snapshot = s_telemetry;
        __DMB();
        after = s_telemetry_generation;
    } while (before != after || (before & 1U) != 0U);
    return snapshot;
}

static void SendPoseReport(uint32_t now_ms)
{
    uint8_t payload[30];
    HostTelemetry telemetry = ReadTelemetry();

    WriteI32(&payload[0], (int32_t)lroundf(telemetry.pose.x_mm * 10.0f));
    WriteI32(&payload[4], (int32_t)lroundf(telemetry.pose.y_mm * 10.0f));
    WriteI32(&payload[8], (int32_t)lroundf(telemetry.pose.yaw_deg * 1000.0f));
    WriteI16(&payload[12], ClampI16(telemetry.pose.vx_mm_s * 10.0f));
    WriteI16(&payload[14], ClampI16(telemetry.pose.vy_mm_s * 10.0f));
    WriteI16(&payload[16], ClampI16(telemetry.pose.yaw_rate_deg_s * 100.0f));
    WriteU32(&payload[18], now_ms);
    WriteU16(&payload[22], telemetry.current_waypoint);
    WriteU16(&payload[24], telemetry.fault_flags);
    payload[26] = telemetry.navigation_state;
    payload[27] = telemetry.pose_state;
    payload[28] = telemetry.motor_valid_mask;
    payload[29] = telemetry.imu_valid;
    (void)QueueFrame(HOST_MESSAGE_POSE_REPORT, s_tx_sequence++, payload,
                     sizeof(payload));
}

void HostLink_Process(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (s_rx_restart_requested)
    {
        (void)HAL_UART_AbortReceive(&huart1);
        s_rx_restart_requested = StartReceive() != HAL_OK;
    }
    ProcessReceive();
    if ((now_ms - s_last_pose_report_ms) >= HOST_POSE_REPORT_PERIOD_MS)
    {
        s_last_pose_report_ms = now_ms;
        SendPoseReport(now_ms);
    }
    StartNextTransmit();
}

void HostLink_SendPose(const NavigationPose *pose)
{
    if (pose == NULL)
    {
        return;
    }
    s_telemetry_generation++;
    __DMB();
    s_telemetry.pose = *pose;
    s_telemetry.pose_state = (uint8_t)Localization_GetStatus().pose_state;
    __DMB();
    s_telemetry_generation++;
}

void HostLink_SetStatus(uint8_t motor_valid_mask,
                        bool imu_valid,
                        uint16_t fault_flags,
                        NavigationState navigation_state,
                        uint16_t current_waypoint)
{
    s_telemetry_generation++;
    __DMB();
    s_telemetry.motor_valid_mask = motor_valid_mask;
    s_telemetry.imu_valid = imu_valid ? 1U : 0U;
    s_telemetry.fault_flags = fault_flags;
    s_telemetry.navigation_state = (uint8_t)navigation_state;
    s_telemetry.current_waypoint = current_waypoint;
    __DMB();
    s_telemetry_generation++;
}

bool HostLink_TakeCommand(HostCommand *command)
{
    return command != NULL && s_command_queue != NULL &&
           osMessageQueueGet(s_command_queue, command, NULL, 0U) == osOK;
}

void HostLink_CompleteCommand(const HostCommand *command,
                              bool success,
                              uint8_t result_code)
{
    if (command == NULL)
    {
        return;
    }
    HostDispatchOutput output;

    taskENTER_CRITICAL();
    output = HostDispatcher_CompleteCommand(&s_dispatcher, command,
                                            success, result_code);
    taskEXIT_CRITICAL();

    QueueDispatchReply(&output);
}

bool HostLink_TakeCommittedRoute(NavigationRoute *route)
{
    bool available;

    taskENTER_CRITICAL();
    available = HostDispatcher_TakeCommittedRoute(&s_dispatcher, route);
    taskEXIT_CRITICAL();
    return available;
}

uint32_t HostLink_GetHeartbeatAgeMs(uint32_t now_ms)
{
    return HostDispatcher_GetHeartbeatAgeMs(&s_dispatcher, now_ms);
}

void HostLink_OnUartError(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        s_rx_restart_requested = true;
        s_tx_busy = false;
    }
}

void HostLink_OnUartTxComplete(UART_HandleTypeDef *huart)
{
    if (huart == &huart1 && s_tx_busy)
    {
        s_tx_read = (uint8_t)((s_tx_read + 1U) % HOST_TX_SLOT_COUNT);
        s_tx_busy = false;
    }
}
