#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "control/chassis/chassis.h"
#include "control/navigation/navigation.h"
#include "devices/motor/zdt_x42s/zdt_x42s.h"
#include "platform/communication/host_dispatcher.h"
#include "platform/communication/host_protocol.h"

#define SIM_CAN_CAPACITY 64U

typedef struct
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];
} SimCanFrame;

FDCAN_HandleTypeDef hfdcan1;

static HostProtocolParser s_parser;
static HostDispatcher s_dispatcher;
static NavigationPose s_pose;
static SimCanFrame s_can_frames[SIM_CAN_CAPACITY];
static size_t s_can_count;
static FDCAN_RxHeaderTypeDef s_rx_header;
static uint8_t s_rx_data[8];
static bool s_rx_pending;
static uint64_t s_start_ms;
static uint32_t s_last_pose_ms;
static uint32_t s_last_nav_tick_ms;
static uint16_t s_tx_sequence;
static bool s_can_commands_reported;
static bool s_heartbeat_stop_reported;

static uint64_t MonotonicMs(void)
{
    struct timespec value;

    (void)clock_gettime(CLOCK_MONOTONIC, &value);
    return (uint64_t)value.tv_sec * 1000U + (uint64_t)value.tv_nsec / 1000000U;
}

uint32_t HAL_GetTick(void)
{
    return (uint32_t)(MonotonicMs() - s_start_ms);
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan,
                                         FDCAN_FilterTypeDef *filter)
{
    (void)hfdcan;
    (void)filter;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
                                               uint32_t non_matching_std,
                                               uint32_t non_matching_ext,
                                               uint32_t reject_remote_std,
                                               uint32_t reject_remote_ext)
{
    (void)hfdcan;
    (void)non_matching_std;
    (void)non_matching_ext;
    (void)reject_remote_std;
    (void)reject_remote_ext;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(
    const FDCAN_HandleTypeDef *hfdcan,
    FDCAN_ProtocolStatusTypeDef *status)
{
    (void)hfdcan;
    memset(status, 0, sizeof(*status));
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
                                                 uint32_t notifications,
                                                 uint32_t buffer_indexes)
{
    (void)hfdcan;
    (void)notifications;
    (void)buffer_indexes;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                FDCAN_TxHeaderTypeDef *header,
                                                uint8_t *data)
{
    SimCanFrame *frame;

    (void)hfdcan;
    if (s_can_count >= SIM_CAN_CAPACITY)
    {
        return HAL_BUSY;
    }
    frame = &s_can_frames[s_can_count++];
    frame->header = *header;
    memcpy(frame->data, data, sizeof(frame->data));
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    return 8U;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t fifo,
                                        FDCAN_RxHeaderTypeDef *header,
                                        uint8_t *data)
{
    (void)hfdcan;
    (void)fifo;
    s_rx_pending = false;
    *header = s_rx_header;
    memcpy(data, s_rx_data, sizeof(s_rx_data));
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t fifo)
{
    (void)hfdcan;
    (void)fifo;
    return s_rx_pending ? 1U : 0U;
}

NavigationPose Localization_GetPose(void)
{
    return s_pose;
}

float Heading_TurnOutputForPose(float target_angle, float current_angle)
{
    return target_angle - current_angle;
}

static bool WriteAll(int fd, const uint8_t *data, size_t length)
{
    while (length > 0U)
    {
        ssize_t written = write(fd, data, length);

        if (written < 0 && errno == EINTR)
        {
            continue;
        }
        if (written <= 0)
        {
            return false;
        }
        data += (size_t)written;
        length -= (size_t)written;
    }
    return true;
}

static void SendHostFrame(HostMessageType type,
                          uint16_t sequence,
                          const uint8_t *payload,
                          uint16_t payload_length)
{
    uint8_t frame[HOST_PROTOCOL_MAX_FRAME_SIZE];
    size_t length = HostProtocol_Encode(type, sequence, payload,
                                        payload_length, frame, sizeof(frame));

    if (length == 0U || !WriteAll(STDOUT_FILENO, frame, length))
    {
        _exit(3);
    }
}

static void SendReply(const HostDispatchOutput *output)
{
    if (output->reply_ready)
    {
        SendHostFrame(output->reply_type, output->reply_sequence,
                      output->reply_payload, sizeof(output->reply_payload));
    }
}

static void WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static void SendPose(uint32_t now_ms)
{
    uint8_t payload[30] = {0};

    WriteU32(&payload[0], (uint32_t)(int32_t)(s_pose.x_mm * 10.0f));
    WriteU32(&payload[4], (uint32_t)(int32_t)(s_pose.y_mm * 10.0f));
    WriteU32(&payload[8], (uint32_t)(int32_t)(s_pose.yaw_deg * 1000.0f));
    WriteU32(&payload[18], now_ms);
    WriteU16(&payload[22], Navigation_GetCurrentWaypoint());
    WriteU16(&payload[24], Navigation_GetFaultFlags());
    payload[26] = (uint8_t)Navigation_GetState();
    payload[27] = 0U;
    payload[28] = 0x0FU;
    payload[29] = 1U;
    SendHostFrame(HOST_MESSAGE_POSE_REPORT, s_tx_sequence++,
                  payload, sizeof(payload));
}

static void CompleteCommand(const HostCommand *command, bool success)
{
    HostDispatchOutput output = HostDispatcher_CompleteCommand(
        &s_dispatcher, command, success, success ? 0U : 1U);

    SendReply(&output);
}

static void ExecuteCommand(const HostCommand *command)
{
    bool success = true;

    switch (command->type)
    {
        case HOST_COMMAND_ROUTE_COMMIT:
        {
            NavigationRoute route;

            success = HostDispatcher_TakeCommittedRoute(&s_dispatcher, &route) &&
                      Navigation_LoadRoute(&route) == NAVIGATION_OK;
            break;
        }
        case HOST_COMMAND_NAV_START:
            success = Navigation_Start() == NAVIGATION_OK;
            break;
        case HOST_COMMAND_NAV_PAUSE:
            Navigation_Pause();
            break;
        case HOST_COMMAND_NAV_RESUME:
            Navigation_Resume();
            break;
        case HOST_COMMAND_NAV_STOP:
            Navigation_Stop();
            break;
        case HOST_COMMAND_SET_POSE:
        case HOST_COMMAND_SET_CALIBRATION:
            break;
    }
    CompleteCommand(command, success);
}

static void OnHostFrame(const HostFrame *frame, void *context)
{
    HostDispatchOutput output;

    (void)context;
    output = HostDispatcher_Handle(&s_dispatcher, frame, HAL_GetTick());
    SendReply(&output);
    if (output.force_pose_report)
    {
        SendPose(HAL_GetTick());
    }
    if (output.command_ready)
    {
        ExecuteCommand(&output.command);
    }
}

static void InjectPosition(uint8_t motor_id, int32_t position_x10_deg)
{
    uint32_t magnitude = position_x10_deg < 0 ?
        (uint32_t)(-(int64_t)position_x10_deg) : (uint32_t)position_x10_deg;

    memset(&s_rx_header, 0, sizeof(s_rx_header));
    memset(s_rx_data, 0, sizeof(s_rx_data));
    s_rx_pending = true;
    s_rx_header.Identifier = (uint32_t)motor_id << 8U;
    s_rx_header.IdType = FDCAN_EXTENDED_ID;
    s_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    s_rx_header.DataLength = FDCAN_DLC_BYTES_7;
    s_rx_data[0] = 0x36U;
    s_rx_data[1] = position_x10_deg < 0 ? 0x01U : 0x00U;
    s_rx_data[2] = (uint8_t)(magnitude >> 24U);
    s_rx_data[3] = (uint8_t)(magnitude >> 16U);
    s_rx_data[4] = (uint8_t)(magnitude >> 8U);
    s_rx_data[5] = (uint8_t)magnitude;
    s_rx_data[6] = 0x6BU;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}

static bool ValidateMotorReplies(void)
{
    int32_t positive;
    int32_t negative;
    uint32_t age;

    InjectPosition(1U, 1234);
    InjectPosition(2U, -4321);
    return ZdtX42s_GetPosition(1U, &positive, &age) && positive == 1234 &&
           ZdtX42s_GetPosition(2U, &negative, &age) && negative == -4321;
}

static bool IsExpectedFrame(const SimCanFrame *frame,
                            uint32_t identifier,
                            const uint8_t *data,
                            uint32_t length)
{
    return frame->header.Identifier == identifier &&
           frame->header.IdType == FDCAN_EXTENDED_ID &&
           frame->header.TxFrameType == FDCAN_DATA_FRAME &&
           frame->header.DataLength == length &&
           frame->header.BitRateSwitch == FDCAN_BRS_OFF &&
           frame->header.FDFormat == FDCAN_CLASSIC_CAN &&
           memcmp(frame->data, data, length) == 0;
}

static bool ValidateMotionFrames(void)
{
    static const uint8_t positive[] = {
        0xF6U, 0x00U, 0x03U, 0xE8U, 0x00U, 0xC8U, 0x01U, 0x6BU
    };
    static const uint8_t negative[] = {
        0xF6U, 0x01U, 0x03U, 0xE8U, 0x00U, 0xC8U, 0x01U, 0x6BU
    };
    static const uint8_t sync[] = {0xFFU, 0x66U, 0x6BU};
    size_t start = s_can_count >= 5U ? s_can_count - 5U : 0U;

    return s_can_count >= 5U &&
           IsExpectedFrame(&s_can_frames[start], 0x100U, positive,
                           FDCAN_DLC_BYTES_8) &&
           IsExpectedFrame(&s_can_frames[start + 1U], 0x200U, positive,
                           FDCAN_DLC_BYTES_8) &&
           IsExpectedFrame(&s_can_frames[start + 2U], 0x300U, negative,
                           FDCAN_DLC_BYTES_8) &&
           IsExpectedFrame(&s_can_frames[start + 3U], 0x400U, negative,
                           FDCAN_DLC_BYTES_8) &&
           IsExpectedFrame(&s_can_frames[start + 4U], 0x000U, sync,
                           FDCAN_DLC_BYTES_3);
}

static bool ValidateStopFrames(void)
{
    static const uint8_t stopped[] = {
        0xF6U, 0x00U, 0x03U, 0xE8U, 0x00U, 0x00U, 0x01U, 0x6BU
    };
    static const uint8_t sync[] = {0xFFU, 0x66U, 0x6BU};
    size_t start = s_can_count >= 5U ? s_can_count - 5U : 0U;

    if (s_can_count < 5U)
    {
        return false;
    }
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        if (!IsExpectedFrame(&s_can_frames[start + i], (i + 1U) << 8U,
                             stopped, FDCAN_DLC_BYTES_8))
        {
            return false;
        }
    }
    return IsExpectedFrame(&s_can_frames[start + 4U], 0U, sync,
                           FDCAN_DLC_BYTES_3);
}

static void Tick(uint32_t now_ms)
{
    bool heartbeat_ok = HostDispatcher_GetHeartbeatAgeMs(&s_dispatcher,
                                                          now_ms) <= 500U;

    if ((now_ms - s_last_nav_tick_ms) >= 5U)
    {
        s_last_nav_tick_ms = now_ms;
        Navigation_SetHealth(true, true, true, heartbeat_ok);
        Navigation_Tick(now_ms);
    }
    if (!s_can_commands_reported && Navigation_GetState() == NAVIGATION_MOVING &&
        ValidateMotionFrames())
    {
        fputs("CAN_COMMANDS_OK\n", stderr);
        fflush(stderr);
        s_can_commands_reported = true;
    }
    if (s_can_commands_reported && heartbeat_ok)
    {
        s_can_count = 0U;
    }
    if (!s_heartbeat_stop_reported && s_can_commands_reported &&
        Navigation_GetState() == NAVIGATION_E_STOP && ValidateStopFrames())
    {
        fputs("HEARTBEAT_STOP_OK\n", stderr);
        fflush(stderr);
        s_heartbeat_stop_reported = true;
    }
    if ((now_ms - s_last_pose_ms) >= 100U)
    {
        s_last_pose_ms = now_ms;
        SendPose(now_ms);
    }
}

int main(void)
{
    uint8_t input[128];
    struct pollfd descriptor = {.fd = STDIN_FILENO, .events = POLLIN};

    s_start_ms = MonotonicMs();
    HostProtocol_Init(&s_parser, OnHostFrame, NULL);
    HostDispatcher_Init(&s_dispatcher, 0U);
    Navigation_Init();
    if (Chassis_Init() != HAL_OK)
    {
        return 2;
    }
    s_can_count = 0U;
    if (!ValidateMotorReplies())
    {
        return 2;
    }
    fputs("MOTOR_REPLIES_OK\n", stderr);
    fflush(stderr);

    for (;;)
    {
        int status = poll(&descriptor, 1U, 5);

        if (status < 0 && errno != EINTR)
        {
            return 2;
        }
        if (status > 0 && (descriptor.revents & POLLIN) != 0)
        {
            ssize_t length = read(STDIN_FILENO, input, sizeof(input));

            if (length <= 0)
            {
                break;
            }
            HostProtocol_Consume(&s_parser, input, (size_t)length);
        }
        if ((descriptor.revents & POLLHUP) != 0)
        {
            break;
        }
        Tick(HAL_GetTick());
    }
    return s_can_commands_reported && s_heartbeat_stop_reported ? 0 : 4;
}
