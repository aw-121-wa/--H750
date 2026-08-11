#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "devices/imu/hwt101/hwt101_parser.h"

static void build_frame(uint8_t type, int16_t z_value, uint8_t frame[IMU_FRAME_SIZE])
{
    uint8_t checksum = 0U;

    memset(frame, 0, IMU_FRAME_SIZE);
    frame[0] = IMU_FRAME_HEADER;
    frame[1] = type;
    frame[6] = (uint8_t)((uint16_t)z_value & 0xFFU);
    frame[7] = (uint8_t)((uint16_t)z_value >> 8U);
    for (uint32_t i = 0U; i < IMU_FRAME_SIZE - 1U; ++i)
    {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    frame[IMU_FRAME_SIZE - 1U] = checksum;
}

static void test_angle_and_rate_frames(void)
{
    Imu sample = {0};
    uint8_t frame[IMU_FRAME_SIZE];

    build_frame(IMU_FRAME_ANGLE, 16384, frame);
    assert(IMU_ParseFrame(frame, &sample) == IMU_PARSE_OK);
    assert(fabsf(sample.yaw - 81.0f) < 0.01f);

    build_frame(IMU_FRAME_ANGULAR_RATE, 16384, frame);
    assert(IMU_ParseFrame(frame, &sample) == IMU_PARSE_OK);
    assert(fabsf(sample.angular_rate - 1000.0f) < 0.01f);
}

static void test_wraparound_uses_shortest_yaw_delta(void)
{
    Imu sample = {.yaw = 179.0f};
    uint8_t frame[IMU_FRAME_SIZE];

    build_frame(IMU_FRAME_ANGLE, -32586, frame);
    assert(IMU_ParseFrame(frame, &sample) == IMU_PARSE_OK);
    assert(sample.yaw < -179.0f && sample.yaw > -179.5f);
}

static void test_invalid_frames_are_rejected(void)
{
    Imu sample = {.yaw = 12.0f, .angular_rate = 34.0f};
    uint8_t frame[IMU_FRAME_SIZE];

    build_frame(IMU_FRAME_ANGLE, 1000, frame);
    frame[0] = 0U;
    assert(IMU_ParseFrame(frame, &sample) == IMU_PARSE_BAD_HEADER);

    build_frame(0x51U, 1000, frame);
    assert(IMU_ParseFrame(frame, &sample) == IMU_PARSE_UNSUPPORTED_FRAME);

    build_frame(IMU_FRAME_ANGLE, 1000, frame);
    frame[10]++;
    assert(IMU_ParseFrame(frame, &sample) == IMU_PARSE_BAD_CHECKSUM);
    assert(sample.yaw == 12.0f && sample.angular_rate == 34.0f);
}

int main(void)
{
    test_angle_and_rate_frames();
    test_wraparound_uses_shortest_yaw_delta();
    test_invalid_frames_are_rejected();
    puts("IMU parser tests passed");
    return 0;
}
