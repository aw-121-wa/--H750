#ifndef DEVICES_IMU_HWT101_PARSER_H
#define DEVICES_IMU_HWT101_PARSER_H

#include <stdint.h>

#define HWT101_FRAME_SIZE         11U
#define HWT101_FRAME_HEADER       0x55U
#define HWT101_FRAME_ANGULAR_RATE 0x52U
#define HWT101_FRAME_ANGLE        0x53U

typedef struct Imu
{
    float yaw;
    float roll;
    float pitch;
    float angular_rate;
} Hwt101Data;

typedef enum
{
    HWT101_PARSE_OK = 0,
    HWT101_PARSE_BAD_HEADER,
    HWT101_PARSE_UNSUPPORTED_FRAME,
    HWT101_PARSE_BAD_CHECKSUM,
    HWT101_PARSE_INVALID_ARGUMENT
} Hwt101ParseResult;

Hwt101ParseResult Hwt101_ParseFrame(
    const uint8_t frame[HWT101_FRAME_SIZE], Hwt101Data *data);

typedef Hwt101Data Imu;
typedef Hwt101ParseResult IMU_ParseResult;
#define IMU_FRAME_SIZE              HWT101_FRAME_SIZE
#define IMU_FRAME_HEADER            HWT101_FRAME_HEADER
#define IMU_FRAME_ANGULAR_RATE      HWT101_FRAME_ANGULAR_RATE
#define IMU_FRAME_ANGLE             HWT101_FRAME_ANGLE
#define IMU_PARSE_OK                HWT101_PARSE_OK
#define IMU_PARSE_BAD_HEADER        HWT101_PARSE_BAD_HEADER
#define IMU_PARSE_UNSUPPORTED_FRAME HWT101_PARSE_UNSUPPORTED_FRAME
#define IMU_PARSE_BAD_CHECKSUM      HWT101_PARSE_BAD_CHECKSUM
#define IMU_PARSE_INVALID_ARGUMENT  HWT101_PARSE_INVALID_ARGUMENT
#define IMU_ParseFrame              Hwt101_ParseFrame

#endif
