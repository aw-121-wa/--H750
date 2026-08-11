#include "control/chassis/chassis.h"

#include "control/chassis/chassis_kinematics.h"
#include "devices/motor/zdt_x42s/zdt_x42s.h"
#include "fdcan.h"

#define CHASSIS_BATCH_FRAME_COUNT 5U
#define CHASSIS_ACCELERATION      0U

HAL_StatusTypeDef Chassis_Init(void)
{
    const int16_t stopped[CHASSIS_WHEEL_COUNT] = {0};
    HAL_StatusTypeDef status = ZdtX42s_Init(&hfdcan1);

    return status == HAL_OK ? Chassis_SetWheelSpeedX10(stopped) : status;
}

HAL_StatusTypeDef Chassis_SetWheelSpeedX10(
    const int16_t rpm_x10[CHASSIS_WHEEL_COUNT])
{
    if (rpm_x10 == NULL)
    {
        return HAL_ERROR;
    }
    if (!ZdtX42s_HasTxCapacity(CHASSIS_BATCH_FRAME_COUNT))
    {
        return HAL_BUSY;
    }

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        HAL_StatusTypeDef status = ZdtX42s_SetSpeedX10(
            i + 1U, rpm_x10[i], CHASSIS_ACCELERATION, true);
        if (status != HAL_OK)
        {
            return status;
        }
    }
    return ZdtX42s_Sync();
}

HAL_StatusTypeDef Chassis_SetSpeed(float vy, float vx, float vw, bool fine)
{
    float wheel_rpm[CHASSIS_WHEEL_COUNT];
    int16_t wheel_rpm_x10[CHASSIS_WHEEL_COUNT];

    Chassis_CalculateWheelRpm(vy, vx, vw, wheel_rpm);
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        wheel_rpm_x10[i] = Chassis_EncodeRpmX10(wheel_rpm[i], fine);
    }
    return Chassis_SetWheelSpeedX10(wheel_rpm_x10);
}

void Motor_setspeed(float vy, float vx, float vw)
{
    (void)Chassis_SetSpeed(vy, vx, vw, false);
}

void Motor_setspeed_fine(float vy, float vx, float vw)
{
    (void)Chassis_SetSpeed(vy, vx, vw, true);
}

void Motor_Stop(void)
{
    (void)Chassis_SetSpeed(0.0f, 0.0f, 0.0f, false);
}
