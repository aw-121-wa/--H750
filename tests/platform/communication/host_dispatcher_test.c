#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "platform/communication/host_dispatcher.h"

static HostFrame MakeFrame(HostMessageType type,
                           uint16_t sequence,
                           const uint8_t *payload,
                           uint16_t length)
{
    HostFrame frame = {.type = type, .sequence = sequence, .length = length};

    if (length > 0U)
    {
        memcpy(frame.payload, payload, length);
    }
    return frame;
}

static void WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void WriteI32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8U);
    data[2] = (uint8_t)(raw >> 16U);
    data[3] = (uint8_t)(raw >> 24U);
}

static void test_immediate_reply_and_heartbeat_age(void)
{
    HostDispatcher dispatcher;
    HostDispatchOutput output;
    HostFrame heartbeat = MakeFrame(HOST_MESSAGE_HEARTBEAT, 4U, NULL, 0U);

    HostDispatcher_Init(&dispatcher, 100U);
    output = HostDispatcher_Handle(&dispatcher, &heartbeat, 125U);

    assert(output.reply_ready);
    assert(output.reply_type == HOST_MESSAGE_ACK);
    assert(output.reply_payload[0] == HOST_MESSAGE_HEARTBEAT);
    assert(!output.command_ready);
    assert(HostDispatcher_GetHeartbeatAgeMs(&dispatcher, 175U) == 50U);
}

static void test_atomic_route_and_duplicate_command(void)
{
    HostDispatcher dispatcher;
    HostDispatchOutput output;
    NavigationRoute route;
    uint8_t route_bytes[8];
    uint8_t begin_payload[4];
    uint8_t chunk_payload[11];
    HostFrame frame;

    WriteI32(&route_bytes[0], 10000);
    WriteI32(&route_bytes[4], 0);
    WriteU16(&begin_payload[0], 1U);
    WriteU16(&begin_payload[2], HostProtocol_Crc16(route_bytes,
                                                   sizeof(route_bytes)));
    WriteU16(&chunk_payload[0], 0U);
    chunk_payload[2] = 1U;
    memcpy(&chunk_payload[3], route_bytes, sizeof(route_bytes));

    HostDispatcher_Init(&dispatcher, 0U);
    frame = MakeFrame(HOST_MESSAGE_ROUTE_BEGIN, 10U,
                      begin_payload, sizeof(begin_payload));
    assert(HostDispatcher_Handle(&dispatcher, &frame, 10U).reply_type ==
           HOST_MESSAGE_ACK);
    frame = MakeFrame(HOST_MESSAGE_ROUTE_CHUNK, 11U,
                      chunk_payload, sizeof(chunk_payload));
    assert(HostDispatcher_Handle(&dispatcher, &frame, 11U).reply_type ==
           HOST_MESSAGE_ACK);
    frame = MakeFrame(HOST_MESSAGE_ROUTE_COMMIT, 12U, NULL, 0U);
    output = HostDispatcher_Handle(&dispatcher, &frame, 12U);
    assert(output.command_ready);
    assert(!output.reply_ready);
    assert(output.command.type == HOST_COMMAND_ROUTE_COMMIT);
    assert(HostDispatcher_TakeCommittedRoute(&dispatcher, &route));
    assert(route.point_count == 1U);
    assert(route.points[0].x_x10_mm == 10000);

    output = HostDispatcher_Handle(&dispatcher, &frame, 13U);
    assert(!output.command_ready);
    assert(!output.reply_ready);

    output = HostDispatcher_CompleteCommand(&dispatcher,
                                            &(HostCommand){
                                                .type = HOST_COMMAND_ROUTE_COMMIT,
                                                .source_type = HOST_MESSAGE_ROUTE_COMMIT,
                                                .sequence = 12U
                                            },
                                            true, 0U);
    assert(output.reply_ready);
    assert(output.reply_type == HOST_MESSAGE_ACK);

    output = HostDispatcher_Handle(&dispatcher, &frame, 14U);
    assert(!output.command_ready);
    assert(output.reply_ready);
    assert(output.reply_type == HOST_MESSAGE_ACK);
}

static void test_rejects_incomplete_route(void)
{
    HostDispatcher dispatcher;
    HostDispatchOutput output;
    const uint8_t begin_payload[] = {1U, 0U, 0U, 0U};
    HostFrame begin = MakeFrame(HOST_MESSAGE_ROUTE_BEGIN, 20U,
                                begin_payload, sizeof(begin_payload));
    HostFrame commit = MakeFrame(HOST_MESSAGE_ROUTE_COMMIT, 21U, NULL, 0U);

    HostDispatcher_Init(&dispatcher, 0U);
    assert(HostDispatcher_Handle(&dispatcher, &begin, 1U).reply_type ==
           HOST_MESSAGE_ACK);
    output = HostDispatcher_Handle(&dispatcher, &commit, 2U);
    assert(output.reply_ready);
    assert(output.reply_type == HOST_MESSAGE_NACK);
    assert(!output.command_ready);
}

int main(void)
{
    test_immediate_reply_and_heartbeat_age();
    test_atomic_route_and_duplicate_command();
    test_rejects_incomplete_route();
    puts("Host dispatcher tests passed");
    return 0;
}
