#ifndef PLATFORM_COMMUNICATION_HOST_PROTOCOL_H
#define PLATFORM_COMMUNICATION_HOST_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define HOST_PROTOCOL_VERSION 1U
#define HOST_PROTOCOL_MAX_PAYLOAD 256U
#define HOST_PROTOCOL_OVERHEAD 10U
#define HOST_PROTOCOL_MAX_FRAME_SIZE \
    (HOST_PROTOCOL_MAX_PAYLOAD + HOST_PROTOCOL_OVERHEAD)

typedef enum
{
    HOST_MESSAGE_HEARTBEAT = 0x01,
    HOST_MESSAGE_GET_STATUS = 0x02,
    HOST_MESSAGE_SET_POSE = 0x03,
    HOST_MESSAGE_SET_CALIBRATION = 0x04,
    HOST_MESSAGE_ROUTE_BEGIN = 0x10,
    HOST_MESSAGE_ROUTE_CHUNK = 0x11,
    HOST_MESSAGE_ROUTE_COMMIT = 0x12,
    HOST_MESSAGE_NAV_START = 0x20,
    HOST_MESSAGE_NAV_PAUSE = 0x21,
    HOST_MESSAGE_NAV_RESUME = 0x22,
    HOST_MESSAGE_NAV_STOP = 0x23,
    HOST_MESSAGE_ACK = 0x80,
    HOST_MESSAGE_NACK = 0x81,
    HOST_MESSAGE_POSE_REPORT = 0x90,
    HOST_MESSAGE_STATUS_REPORT = 0x91,
    HOST_MESSAGE_FAULT_REPORT = 0x92
} HostMessageType;

typedef struct
{
    HostMessageType type;
    uint16_t sequence;
    uint16_t length;
    uint8_t payload[HOST_PROTOCOL_MAX_PAYLOAD];
} HostFrame;

typedef void (*HostFrameCallback)(const HostFrame *frame, void *context);

typedef struct
{
    uint8_t buffer[HOST_PROTOCOL_MAX_FRAME_SIZE];
    size_t used;
    uint32_t crc_error_count;
    uint32_t length_error_count;
    HostFrameCallback callback;
    void *context;
} HostProtocolParser;

/** Computes CRC16-CCITT with initial value 0xFFFF. */
uint16_t HostProtocol_Crc16(const uint8_t *data, size_t length);

/** Encodes one host frame and returns its byte count, or zero on error. */
size_t HostProtocol_Encode(HostMessageType type,
                           uint16_t sequence,
                           const uint8_t *payload,
                           uint16_t payload_length,
                           uint8_t *output,
                           size_t output_capacity);

/** Initializes an incremental byte-stream parser. */
void HostProtocol_Init(HostProtocolParser *parser,
                       HostFrameCallback callback,
                       void *context);

/** Consumes arbitrary stream chunks and emits complete valid frames. */
void HostProtocol_Consume(HostProtocolParser *parser,
                          const uint8_t *data,
                          size_t length);

#endif
