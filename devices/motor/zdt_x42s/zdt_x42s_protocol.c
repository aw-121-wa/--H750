#include "devices/motor/zdt_x42s/zdt_x42s_protocol.h"

uint32_t ZdtX42s_CommandId(uint8_t motor_id, uint8_t packet_index)
{
    return ((uint32_t)motor_id << 8U) | packet_index;
}

size_t ZdtX42s_BuildSpeedPayload(
    int16_t rpm_x10,
    uint16_t acceleration_rpm_s,
    bool synchronized,
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH])
{
    uint16_t magnitude;

    if (payload == NULL)
    {
        return 0U;
    }

    if (rpm_x10 < 0)
    {
        magnitude = (uint16_t)(-(int32_t)rpm_x10);
        payload[1] = 0x01U;
    }
    else
    {
        magnitude = (uint16_t)rpm_x10;
        payload[1] = 0x00U;
    }

    payload[0] = 0xF6U;
    payload[2] = (uint8_t)(acceleration_rpm_s >> 8U);
    payload[3] = (uint8_t)acceleration_rpm_s;
    payload[4] = (uint8_t)(magnitude >> 8U);
    payload[5] = (uint8_t)magnitude;
    payload[6] = synchronized ? 0x01U : 0x00U;
    payload[7] = 0x6BU;
    return 8U;
}

size_t ZdtX42s_BuildSyncPayload(
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH])
{
    if (payload == NULL)
    {
        return 0U;
    }

    payload[0] = 0xFFU;
    payload[1] = 0x66U;
    payload[2] = 0x6BU;
    for (size_t i = 3U; i < ZDT_X42S_CAN_MAX_DATA_LENGTH; ++i)
    {
        payload[i] = 0x00U;
    }
    return 3U;
}
