#include "platform/communication/host_dispatcher.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define HOST_ROUTE_POINT_BYTES 8U

static uint16_t ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static int32_t ReadI32(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0] |
                     ((uint32_t)data[1] << 8U) |
                     ((uint32_t)data[2] << 16U) |
                     ((uint32_t)data[3] << 24U));
}

static void WriteI32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8U);
    data[2] = (uint8_t)(raw >> 16U);
    data[3] = (uint8_t)(raw >> 24U);
}

static HostDispatchOutput Reply(const HostFrame *frame,
                                bool success,
                                uint8_t code)
{
    HostDispatchOutput output = {0};

    output.reply_ready = true;
    output.reply_type = success ? HOST_MESSAGE_ACK : HOST_MESSAGE_NACK;
    output.reply_sequence = frame->sequence;
    output.reply_payload[0] = (uint8_t)frame->type;
    output.reply_payload[1] = code;
    return output;
}

static void CacheReply(HostDispatcher *dispatcher,
                       const HostCommand *command,
                       bool success,
                       uint8_t code)
{
    HostReplyCacheEntry *entry =
        &dispatcher->reply_cache[dispatcher->reply_cache_write];

    entry->sequence = command->sequence;
    entry->type = (HostMessageType)command->source_type;
    entry->code = code;
    entry->success = success;
    entry->valid = true;
    dispatcher->reply_cache_write = (uint8_t)(
        (dispatcher->reply_cache_write + 1U) %
        (sizeof(dispatcher->reply_cache) / sizeof(dispatcher->reply_cache[0])));
}

static bool IsPending(const HostDispatcher *dispatcher,
                      const HostFrame *frame)
{
    for (size_t i = 0U; i < sizeof(dispatcher->pending) /
                              sizeof(dispatcher->pending[0]); ++i)
    {
        if (dispatcher->pending[i].valid &&
            dispatcher->pending[i].sequence == frame->sequence &&
            dispatcher->pending[i].type == frame->type)
        {
            return true;
        }
    }
    return false;
}

static bool AddPending(HostDispatcher *dispatcher, const HostFrame *frame)
{
    for (size_t i = 0U; i < sizeof(dispatcher->pending) /
                              sizeof(dispatcher->pending[0]); ++i)
    {
        if (!dispatcher->pending[i].valid)
        {
            dispatcher->pending[i].sequence = frame->sequence;
            dispatcher->pending[i].type = frame->type;
            dispatcher->pending[i].valid = true;
            return true;
        }
    }
    return false;
}

static void RemovePending(HostDispatcher *dispatcher,
                          const HostCommand *command)
{
    for (size_t i = 0U; i < sizeof(dispatcher->pending) /
                              sizeof(dispatcher->pending[0]); ++i)
    {
        if (dispatcher->pending[i].valid &&
            dispatcher->pending[i].sequence == command->sequence &&
            dispatcher->pending[i].type ==
                (HostMessageType)command->source_type)
        {
            dispatcher->pending[i].valid = false;
            return;
        }
    }
}

static bool DecodePose(const HostFrame *frame, HostCommand *command)
{
    if (frame->length != 12U)
    {
        return false;
    }
    command->type = HOST_COMMAND_SET_POSE;
    command->data.pose.x_mm = (float)ReadI32(&frame->payload[0]) * 0.1f;
    command->data.pose.y_mm = (float)ReadI32(&frame->payload[4]) * 0.1f;
    command->data.pose.yaw_deg = (float)ReadI32(&frame->payload[8]) * 0.001f;
    command->data.pose.vx_mm_s = 0.0f;
    command->data.pose.vy_mm_s = 0.0f;
    command->data.pose.yaw_rate_deg_s = 0.0f;
    return command->data.pose.x_mm >= 0.0f &&
           command->data.pose.x_mm <= 10000.0f &&
           command->data.pose.y_mm >= 0.0f &&
           command->data.pose.y_mm <= 10000.0f &&
           command->data.pose.yaw_deg >= -180.0f &&
           command->data.pose.yaw_deg <= 180.0f;
}

static bool DecodeCalibration(const HostFrame *frame, HostCommand *command)
{
    if (frame->length != 12U)
    {
        return false;
    }
    command->type = HOST_COMMAND_SET_CALIBRATION;
    memcpy(&command->data.calibration.longitudinal_mm_per_x10_deg,
           &frame->payload[0], sizeof(float));
    memcpy(&command->data.calibration.lateral_mm_per_x10_deg,
           &frame->payload[4], sizeof(float));
    memcpy(&command->data.calibration.yaw_scale,
           &frame->payload[8], sizeof(float));
    return isfinite(command->data.calibration.longitudinal_mm_per_x10_deg) &&
           isfinite(command->data.calibration.lateral_mm_per_x10_deg) &&
           isfinite(command->data.calibration.yaw_scale);
}

static bool HandleRouteBegin(HostDispatcher *dispatcher,
                             const HostFrame *frame)
{
    uint16_t count;

    if (frame->length != 4U || dispatcher->committed_route_pending)
    {
        return false;
    }
    count = ReadU16(frame->payload);
    if (count == 0U || count > NAVIGATION_MAX_WAYPOINTS)
    {
        return false;
    }
    memset(&dispatcher->staging_route, 0, sizeof(dispatcher->staging_route));
    memset(dispatcher->staging_received, 0,
           sizeof(dispatcher->staging_received));
    dispatcher->staging_route.point_count = count;
    dispatcher->staging_crc = ReadU16(&frame->payload[2]);
    return true;
}

static bool HandleRouteChunk(HostDispatcher *dispatcher,
                             const HostFrame *frame)
{
    uint16_t start;
    uint8_t count;

    if (frame->length < 3U || dispatcher->staging_route.point_count == 0U)
    {
        return false;
    }
    start = ReadU16(frame->payload);
    count = frame->payload[2];
    if (count == 0U || frame->length != 3U + count * HOST_ROUTE_POINT_BYTES ||
        (uint32_t)start + count > dispatcher->staging_route.point_count)
    {
        return false;
    }
    for (uint8_t i = 0U; i < count; ++i)
    {
        const uint8_t *point = &frame->payload[3U + i * HOST_ROUTE_POINT_BYTES];

        dispatcher->staging_route.points[start + i].x_x10_mm = ReadI32(point);
        dispatcher->staging_route.points[start + i].y_x10_mm =
            ReadI32(&point[4]);
        dispatcher->staging_received[start + i] = 1U;
    }
    return true;
}

static bool HandleRouteCommit(HostDispatcher *dispatcher)
{
    uint16_t byte_count;

    if (dispatcher->staging_route.point_count == 0U ||
        dispatcher->committed_route_pending)
    {
        return false;
    }
    for (uint16_t i = 0U; i < dispatcher->staging_route.point_count; ++i)
    {
        const NavigationWaypoint *point = &dispatcher->staging_route.points[i];

        if (dispatcher->staging_received[i] == 0U ||
            point->x_x10_mm < 0 || point->x_x10_mm > 100000 ||
            point->y_x10_mm < 0 || point->y_x10_mm > 100000)
        {
            return false;
        }
        WriteI32(&dispatcher->route_crc_bytes[i * HOST_ROUTE_POINT_BYTES],
                 point->x_x10_mm);
        WriteI32(&dispatcher->route_crc_bytes[i * HOST_ROUTE_POINT_BYTES + 4U],
                 point->y_x10_mm);
    }
    byte_count = (uint16_t)(dispatcher->staging_route.point_count *
                            HOST_ROUTE_POINT_BYTES);
    if (HostProtocol_Crc16(dispatcher->route_crc_bytes, byte_count) !=
        dispatcher->staging_crc)
    {
        return false;
    }
    dispatcher->committed_route = dispatcher->staging_route;
    dispatcher->committed_route_pending = true;
    return true;
}

void HostDispatcher_Init(HostDispatcher *dispatcher, uint32_t now_ms)
{
    if (dispatcher == NULL)
    {
        return;
    }
    memset(dispatcher, 0, sizeof(*dispatcher));
    dispatcher->last_heartbeat_ms = now_ms;
}

HostDispatchOutput HostDispatcher_Handle(HostDispatcher *dispatcher,
                                         const HostFrame *frame,
                                         uint32_t now_ms)
{
    HostDispatchOutput output = {0};
    HostCommand command = {0};
    bool valid = false;
    bool delayed = false;

    if (dispatcher == NULL || frame == NULL)
    {
        return output;
    }
    dispatcher->last_heartbeat_ms = now_ms;
    for (size_t i = 0U; i < sizeof(dispatcher->reply_cache) /
                              sizeof(dispatcher->reply_cache[0]); ++i)
    {
        const HostReplyCacheEntry *entry = &dispatcher->reply_cache[i];

        if (entry->valid && entry->sequence == frame->sequence &&
            entry->type == frame->type)
        {
            return Reply(frame, entry->success, entry->code);
        }
    }
    if (IsPending(dispatcher, frame))
    {
        return output;
    }

    command.sequence = frame->sequence;
    command.source_type = (uint8_t)frame->type;
    switch (frame->type)
    {
        case HOST_MESSAGE_HEARTBEAT:
            valid = frame->length == 0U;
            break;
        case HOST_MESSAGE_GET_STATUS:
            valid = frame->length == 0U;
            output.force_pose_report = valid;
            break;
        case HOST_MESSAGE_SET_POSE:
            valid = DecodePose(frame, &command);
            delayed = valid;
            break;
        case HOST_MESSAGE_SET_CALIBRATION:
            valid = DecodeCalibration(frame, &command);
            delayed = valid;
            break;
        case HOST_MESSAGE_ROUTE_BEGIN:
            valid = HandleRouteBegin(dispatcher, frame);
            break;
        case HOST_MESSAGE_ROUTE_CHUNK:
            valid = HandleRouteChunk(dispatcher, frame);
            break;
        case HOST_MESSAGE_ROUTE_COMMIT:
            command.type = HOST_COMMAND_ROUTE_COMMIT;
            valid = frame->length == 0U && HandleRouteCommit(dispatcher);
            delayed = valid;
            break;
        case HOST_MESSAGE_NAV_START:
            command.type = HOST_COMMAND_NAV_START;
            valid = frame->length == 0U;
            delayed = valid;
            break;
        case HOST_MESSAGE_NAV_PAUSE:
            command.type = HOST_COMMAND_NAV_PAUSE;
            valid = frame->length == 0U;
            delayed = valid;
            break;
        case HOST_MESSAGE_NAV_RESUME:
            command.type = HOST_COMMAND_NAV_RESUME;
            valid = frame->length == 0U;
            delayed = valid;
            break;
        case HOST_MESSAGE_NAV_STOP:
            command.type = HOST_COMMAND_NAV_STOP;
            valid = frame->length == 0U;
            delayed = valid;
            break;
        default:
            break;
    }

    if (delayed && AddPending(dispatcher, frame))
    {
        output.command_ready = true;
        output.command = command;
        return output;
    }
    if (delayed)
    {
        valid = false;
        if (frame->type == HOST_MESSAGE_ROUTE_COMMIT)
        {
            dispatcher->committed_route_pending = false;
        }
    }
    output = Reply(frame, valid, valid ? 0U : 1U);
    output.force_pose_report = frame->type == HOST_MESSAGE_GET_STATUS && valid;
    return output;
}

HostDispatchOutput HostDispatcher_CompleteCommand(HostDispatcher *dispatcher,
                                                  const HostCommand *command,
                                                  bool success,
                                                  uint8_t result_code)
{
    HostFrame frame;

    if (dispatcher == NULL || command == NULL)
    {
        return (HostDispatchOutput){0};
    }
    RemovePending(dispatcher, command);
    if (!success && command->type == HOST_COMMAND_ROUTE_COMMIT)
    {
        dispatcher->committed_route_pending = false;
    }
    CacheReply(dispatcher, command, success, result_code);
    memset(&frame, 0, sizeof(frame));
    frame.type = (HostMessageType)command->source_type;
    frame.sequence = command->sequence;
    return Reply(&frame, success, result_code);
}

bool HostDispatcher_TakeCommittedRoute(HostDispatcher *dispatcher,
                                       NavigationRoute *route)
{
    if (dispatcher == NULL || route == NULL ||
        !dispatcher->committed_route_pending)
    {
        return false;
    }
    *route = dispatcher->committed_route;
    dispatcher->committed_route_pending = false;
    return true;
}

uint32_t HostDispatcher_GetHeartbeatAgeMs(const HostDispatcher *dispatcher,
                                          uint32_t now_ms)
{
    return dispatcher == NULL ? UINT32_MAX :
                                now_ms - dispatcher->last_heartbeat_ms;
}
