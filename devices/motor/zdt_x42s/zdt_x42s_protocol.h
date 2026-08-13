#ifndef DEVICES_MOTOR_ZDT_X42S_PROTOCOL_H
#define DEVICES_MOTOR_ZDT_X42S_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ZDT_X42S_CAN_MAX_DATA_LENGTH 8U

/** 将电机地址和包索引编码为扩展 CAN ID。 */
uint32_t ZdtX42s_CommandId(uint8_t motor_id, uint8_t packet_index);

/** 构建七字节 X42S 速度命令载荷。 */
size_t ZdtX42s_BuildSpeedPayload(
    int16_t rpm_x10,
    uint16_t acceleration_rpm_s,
    bool synchronized,
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH]);

/** 构建三字节同步启动载荷。 */
size_t ZdtX42s_BuildSyncPayload(
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH]);

#endif
