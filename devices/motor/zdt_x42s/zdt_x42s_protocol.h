#ifndef DEVICES_MOTOR_ZDT_X42S_PROTOCOL_H
#define DEVICES_MOTOR_ZDT_X42S_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ZDT_X42S_CAN_MAX_DATA_LENGTH 8U

uint32_t ZdtX42s_CommandId(uint8_t motor_id, uint8_t packet_index);
size_t ZdtX42s_BuildSpeedPayload(
    int16_t rpm_x10,
    uint8_t acceleration,
    bool synchronized,
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH]);
size_t ZdtX42s_BuildSyncPayload(
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH]);

#define ZDT_X42S_CommandId         ZdtX42s_CommandId
#define ZDT_X42S_BuildSpeedPayload ZdtX42s_BuildSpeedPayload
#define ZDT_X42S_BuildSyncPayload  ZdtX42s_BuildSyncPayload

#endif
