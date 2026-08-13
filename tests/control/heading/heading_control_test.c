#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "devices/imu/hwt101/hwt101.h"
#include "control/heading/heading_control.h"

struct Imu imu;

static HAL_StatusTypeDef fake_imu_init_status;
static float captured_vw;
static bool captured_fine;
static unsigned int chassis_command_count;
static unsigned int stop_count;

HAL_StatusTypeDef IMU_Receive_Init(void)
{
    return fake_imu_init_status;
}

HAL_StatusTypeDef Chassis_SetSpeed(float vy, float vx, float vw, bool fine)
{
    assert(vy == 0.0f && vx == 0.0f);
    captured_vw = vw;
    captured_fine = fine;
    chassis_command_count++;
    return HAL_OK;
}

HAL_StatusTypeDef Chassis_Stop(void)
{
    stop_count++;
    return HAL_OK;
}

static void reset_fake(void)
{
    imu = (struct Imu){0};
    fake_imu_init_status = HAL_OK;
    captured_vw = 0.0f;
    captured_fine = false;
    chassis_command_count = 0U;
    stop_count = 0U;
}

static void test_init_and_renamed_run_state(void)
{
    reset_fake();
    assert(Gyro_Init() == HAL_OK);
    assert(fabsf(Gyro_Pid.kp - 2.1f) < 0.001f);

    Imu_run.SPEED = 12.0f;
    assert(Imu_run.SPEED == 12.0f);
}

static void test_turn_uses_shortest_path_across_wrap(void)
{
    float output;

    reset_fake();
    assert(Gyro_Init() == HAL_OK);
    imu.yaw = 179.0f;
    output = Direction_Calibration_turn(-179.0f);
    assert(output < -5.1f && output > -5.3f);
}

static void test_calibration_drives_or_stops(void)
{
    reset_fake();
    assert(Gyro_Init() == HAL_OK);
    imu.yaw = 0.0f;
    Direction_Calibration(10);
    assert(chassis_command_count == 1U);
    assert(captured_vw < 0.0f);
    assert(captured_fine);

    imu.yaw = 9.7f;
    Direction_Calibration(10);
    assert(stop_count == 1U);
}

int main(void)
{
    test_init_and_renamed_run_state();
    test_turn_uses_shortest_path_across_wrap();
    test_calibration_drives_or_stops();
    puts("IMU control tests passed");
    return 0;
}
