#include "platform/communication/host_protocol.h"

#include <string.h>

#define HOST_HEADER_0 0xAAU
#define HOST_HEADER_1 0x55U

static uint16_t ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

uint16_t HostProtocol_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL && length != 0U)
    {
        return 0U;
    }
    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint32_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x8000U) != 0U ?
                  (uint16_t)((crc << 1U) ^ 0x1021U) :
                  (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

size_t HostProtocol_Encode(HostMessageType type,
                           uint16_t sequence,
                           const uint8_t *payload,
                           uint16_t payload_length,
                           uint8_t *output,
                           size_t output_capacity)
{
    size_t frame_length = (size_t)payload_length + HOST_PROTOCOL_OVERHEAD;
    uint16_t crc;

    if (output == NULL || payload_length > HOST_PROTOCOL_MAX_PAYLOAD ||
        output_capacity < frame_length ||
        (payload == NULL && payload_length != 0U))
    {
        return 0U;
    }
    output[0] = HOST_HEADER_0;
    output[1] = HOST_HEADER_1;
    output[2] = HOST_PROTOCOL_VERSION;
    output[3] = (uint8_t)type;
    WriteU16(&output[4], sequence);
    WriteU16(&output[6], payload_length);
    if (payload_length != 0U)
    {
        memcpy(&output[8], payload, payload_length);
    }
    crc = HostProtocol_Crc16(&output[2], 6U + payload_length);
    WriteU16(&output[8U + payload_length], crc);
    return frame_length;
}

void HostProtocol_Init(HostProtocolParser *parser,
                       HostFrameCallback callback,
                       void *context)
{
    if (parser == NULL)
    {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->callback = callback;
    parser->context = context;
}

static void DiscardPrefix(HostProtocolParser *parser, size_t count)
{
    if (count >= parser->used)
    {
        parser->used = 0U;
        return;
    }
    memmove(parser->buffer, &parser->buffer[count], parser->used - count);
    parser->used -= count;
}

static void ParseBufferedFrames(HostProtocolParser *parser)
{
    while (parser->used >= 2U)
    {
        uint16_t payload_length;
        size_t frame_length;
        uint16_t expected_crc;
        uint16_t actual_crc;
        HostFrame frame;

        if (parser->buffer[0] != HOST_HEADER_0 ||
            parser->buffer[1] != HOST_HEADER_1)
        {
            DiscardPrefix(parser, 1U);
            continue;
        }
        if (parser->used < 8U)
        {
            return;
        }
        payload_length = ReadU16(&parser->buffer[6]);
        if (payload_length > HOST_PROTOCOL_MAX_PAYLOAD)
        {
            parser->length_error_count++;
            DiscardPrefix(parser, 2U);
            continue;
        }
        frame_length = (size_t)payload_length + HOST_PROTOCOL_OVERHEAD;
        if (parser->used < frame_length)
        {
            return;
        }
        if (parser->buffer[2] != HOST_PROTOCOL_VERSION)
        {
            DiscardPrefix(parser, frame_length);
            continue;
        }
        expected_crc = ReadU16(&parser->buffer[8U + payload_length]);
        actual_crc = HostProtocol_Crc16(&parser->buffer[2],
                                        6U + payload_length);
        if (actual_crc != expected_crc)
        {
            parser->crc_error_count++;
            DiscardPrefix(parser, 1U);
            continue;
        }

        frame.type = (HostMessageType)parser->buffer[3];
        frame.sequence = ReadU16(&parser->buffer[4]);
        frame.length = payload_length;
        if (payload_length != 0U)
        {
            memcpy(frame.payload, &parser->buffer[8], payload_length);
        }
        if (parser->callback != NULL)
        {
            parser->callback(&frame, parser->context);
        }
        DiscardPrefix(parser, frame_length);
    }
}

void HostProtocol_Consume(HostProtocolParser *parser,
                          const uint8_t *data,
                          size_t length)
{
    if (parser == NULL || (data == NULL && length != 0U))
    {
        return;
    }
    for (size_t i = 0U; i < length; ++i)
    {
        if (parser->used == sizeof(parser->buffer))
        {
            DiscardPrefix(parser, 1U);
            parser->length_error_count++;
        }
        parser->buffer[parser->used++] = data[i];
        ParseBufferedFrames(parser);
    }
}
