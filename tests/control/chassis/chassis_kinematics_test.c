#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "control/chassis/chassis_kinematics.h"
#include "devices/motor/zdt_x42s/zdt_x42s_protocol.h"

static void test_extended_id_mapping(void)
{
    assert(ZDT_X42S_CommandId(1U, 0U) == 0x100U);
    assert(ZDT_X42S_CommandId(4U, 0U) == 0x400U);
    assert(ZDT_X42S_CommandId(4U, 1U) == 0x401U);
}

static void test_speed_payload_direction_and_magnitude(void)
{
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH] = {0};
    const uint8_t expected_positive[] = {0xF6, 0x00, 0x04, 0xD2, 0x07, 0x01, 0x6B};
    const uint8_t expected_negative[] = {0xF6, 0x01, 0x04, 0xD2, 0x07, 0x00, 0x6B};

    assert(ZDT_X42S_BuildSpeedPayload(1234, 7U, true, payload) == 7U);
    assert(memcmp(payload, expected_positive, sizeof(expected_positive)) == 0);

    memset(payload, 0, sizeof(payload));
    assert(ZDT_X42S_BuildSpeedPayload(-1234, 7U, false, payload) == 7U);
    assert(memcmp(payload, expected_negative, sizeof(expected_negative)) == 0);
}

static void test_speed_payload_handles_int16_min(void)
{
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH] = {0};
    const uint8_t expected[] = {0xF6, 0x01, 0x80, 0x00, 0x00, 0x01, 0x6B};

    assert(ZDT_X42S_BuildSpeedPayload(INT16_MIN, 0U, true, payload) == 7U);
    assert(memcmp(payload, expected, sizeof(expected)) == 0);
}

static void test_sync_payload(void)
{
    uint8_t payload[ZDT_X42S_CAN_MAX_DATA_LENGTH] = {0};
    const uint8_t expected[] = {0xFF, 0x66, 0x6B};

    assert(ZDT_X42S_BuildSyncPayload(payload) == 3U);
    assert(memcmp(payload, expected, sizeof(expected)) == 0);
}

static void test_chassis_wheel_mapping(void)
{
    float wheel_rpm[CHASSIS_WHEEL_COUNT] = {0};

    Chassis_CalculateWheelRpm(20.0f, 0.0f, 0.0f, wheel_rpm);
    assert(wheel_rpm[0] == 20.0f && wheel_rpm[1] == 20.0f);
    assert(wheel_rpm[2] == -20.0f && wheel_rpm[3] == -20.0f);

    Chassis_CalculateWheelRpm(0.0f, 20.0f, 0.0f, wheel_rpm);
    assert(wheel_rpm[0] == -20.0f && wheel_rpm[1] == 20.0f);
    assert(wheel_rpm[2] == 20.0f && wheel_rpm[3] == -20.0f);

    Chassis_CalculateWheelRpm(0.0f, 0.0f, 20.0f, wheel_rpm);
    assert(wheel_rpm[0] == 20.0f && wheel_rpm[1] == 20.0f);
    assert(wheel_rpm[2] == 20.0f && wheel_rpm[3] == 20.0f);
}

static void test_rpm_encoding_and_clamping(void)
{
    assert(Chassis_EncodeRpmX10(12.9f, false) == 120);
    assert(Chassis_EncodeRpmX10(-12.9f, false) == -120);
    assert(Chassis_EncodeRpmX10(12.34f, true) == 123);
    assert(Chassis_EncodeRpmX10(-12.36f, true) == -124);
    assert(Chassis_EncodeRpmX10(500.0f, true) == 3000);
    assert(Chassis_EncodeRpmX10(-500.0f, true) == -3000);
}

int main(void)
{
    test_extended_id_mapping();
    test_speed_payload_direction_and_magnitude();
    test_speed_payload_handles_int16_min();
    test_sync_payload();
    test_chassis_wheel_mapping();
    test_rpm_encoding_and_clamping();
    puts("chassis protocol tests passed");
    return 0;
}
