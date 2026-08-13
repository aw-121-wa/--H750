#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "platform/communication/host_protocol.h"

static HostFrame received;
static unsigned receive_count;

static void on_frame(const HostFrame *frame, void *context)
{
    (void)context;
    received = *frame;
    receive_count++;
}

static void test_round_trip_and_noise_recovery(void)
{
    HostProtocolParser parser;
    const uint8_t payload[] = {1U, 2U, 3U, 4U};
    uint8_t encoded[HOST_PROTOCOL_MAX_FRAME_SIZE];
    size_t length;
    const uint8_t noise[] = {0x00U, 0xAAU, 0x11U};

    HostProtocol_Init(&parser, on_frame, NULL);
    length = HostProtocol_Encode(HOST_MESSAGE_SET_POSE, 42U, payload,
                                 sizeof(payload), encoded, sizeof(encoded));
    assert(length > sizeof(payload));
    HostProtocol_Consume(&parser, noise, sizeof(noise));
    HostProtocol_Consume(&parser, encoded, 3U);
    HostProtocol_Consume(&parser, encoded + 3U, length - 3U);

    assert(receive_count == 1U);
    assert(received.type == HOST_MESSAGE_SET_POSE);
    assert(received.sequence == 42U);
    assert(received.length == sizeof(payload));
    assert(memcmp(received.payload, payload, sizeof(payload)) == 0);
}

static void test_rejects_bad_crc(void)
{
    HostProtocolParser parser;
    uint8_t encoded[HOST_PROTOCOL_MAX_FRAME_SIZE];
    size_t length;

    receive_count = 0U;
    HostProtocol_Init(&parser, on_frame, NULL);
    length = HostProtocol_Encode(HOST_MESSAGE_HEARTBEAT, 7U, NULL, 0U,
                                 encoded, sizeof(encoded));
    encoded[length - 1U] ^= 0xFFU;
    HostProtocol_Consume(&parser, encoded, length);
    assert(receive_count == 0U);
    assert(parser.crc_error_count == 1U);
}

int main(void)
{
    test_round_trip_and_noise_recovery();
    test_rejects_bad_crc();
    puts("Host protocol tests passed");
    return 0;
}
