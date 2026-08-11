#include "devices/imu/hwt101/hwt101_parser.h"

#include <stddef.h>

#define HWT101_YAW_FILTER_WEIGHT 0.9f

static float Hwt101_NormalizeYaw(float yaw)
{
    while (yaw > 180.0f)
    {
        yaw -= 360.0f;
    }
    while (yaw < -180.0f)
    {
        yaw += 360.0f;
    }
    return yaw;
}

Hwt101ParseResult Hwt101_ParseFrame(
    const uint8_t frame[HWT101_FRAME_SIZE], Hwt101Data *data)
{
    uint8_t checksum = 0U;
    int16_t raw_z;

    if (frame == NULL || data == NULL)
    {
        return HWT101_PARSE_INVALID_ARGUMENT;
    }
    if (frame[0] != HWT101_FRAME_HEADER)
    {
        return HWT101_PARSE_BAD_HEADER;
    }
    if (frame[1] != HWT101_FRAME_ANGLE &&
        frame[1] != HWT101_FRAME_ANGULAR_RATE)
    {
        return HWT101_PARSE_UNSUPPORTED_FRAME;
    }

    for (uint32_t i = 0U; i < HWT101_FRAME_SIZE - 1U; ++i)
    {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    if (checksum != frame[HWT101_FRAME_SIZE - 1U])
    {
        return HWT101_PARSE_BAD_CHECKSUM;
    }

    raw_z = (int16_t)(((uint16_t)frame[7] << 8U) | frame[6]);
    if (frame[1] == HWT101_FRAME_ANGLE)
    {
        const float new_yaw = (float)raw_z * (180.0f / 32768.0f);
        const float delta = Hwt101_NormalizeYaw(new_yaw - data->yaw);
        data->yaw = Hwt101_NormalizeYaw(
            data->yaw + HWT101_YAW_FILTER_WEIGHT * delta);
    }
    else
    {
        data->angular_rate = (float)raw_z * (2000.0f / 32768.0f);
    }
    return HWT101_PARSE_OK;
}
