#ifndef PLATFORM_COMMUNICATION_HOST_DISPATCHER_H
#define PLATFORM_COMMUNICATION_HOST_DISPATCHER_H

#include <stdbool.h>
#include <stdint.h>

#include "control/localization/localization.h"
#include "control/navigation/navigation.h"
#include "platform/communication/host_protocol.h"

typedef enum
{
    HOST_COMMAND_SET_POSE,
    HOST_COMMAND_SET_CALIBRATION,
    HOST_COMMAND_ROUTE_COMMIT,
    HOST_COMMAND_NAV_START,
    HOST_COMMAND_NAV_PAUSE,
    HOST_COMMAND_NAV_RESUME,
    HOST_COMMAND_NAV_STOP
} HostCommandType;

typedef struct
{
    HostCommandType type;
    uint8_t source_type;
    uint16_t sequence;
    union
    {
        NavigationPose pose;
        LocalizationCalibration calibration;
    } data;
} HostCommand;

typedef struct
{
    uint16_t sequence;
    HostMessageType type;
    uint8_t code;
    bool success;
    bool valid;
} HostReplyCacheEntry;

typedef struct
{
    uint16_t sequence;
    HostMessageType type;
    bool valid;
} HostPendingEntry;

typedef struct
{
    NavigationRoute staging_route;
    NavigationRoute committed_route;
    uint8_t staging_received[NAVIGATION_MAX_WAYPOINTS];
    uint8_t route_crc_bytes[NAVIGATION_MAX_WAYPOINTS * 8U];
    uint16_t staging_crc;
    bool committed_route_pending;
    HostReplyCacheEntry reply_cache[16];
    HostPendingEntry pending[8];
    uint8_t reply_cache_write;
    uint32_t last_heartbeat_ms;
} HostDispatcher;

typedef struct
{
    bool command_ready;
    HostCommand command;
    bool reply_ready;
    HostMessageType reply_type;
    uint16_t reply_sequence;
    uint8_t reply_payload[2];
    bool force_pose_report;
} HostDispatchOutput;

/** Initializes protocol-independent host command state. */
void HostDispatcher_Init(HostDispatcher *dispatcher, uint32_t now_ms);

/** Validates one decoded frame and produces a command or immediate reply. */
HostDispatchOutput HostDispatcher_Handle(HostDispatcher *dispatcher,
                                         const HostFrame *frame,
                                         uint32_t now_ms);

/** Records command completion and produces its final ACK or NACK. */
HostDispatchOutput HostDispatcher_CompleteCommand(HostDispatcher *dispatcher,
                                                  const HostCommand *command,
                                                  bool success,
                                                  uint8_t result_code);

/** Copies a committed route exactly once. */
bool HostDispatcher_TakeCommittedRoute(HostDispatcher *dispatcher,
                                       NavigationRoute *route);

/** Returns time since the latest valid parsed host frame. */
uint32_t HostDispatcher_GetHeartbeatAgeMs(const HostDispatcher *dispatcher,
                                          uint32_t now_ms);

#endif
