#include "control/chassis/chassis_kinematics.h"

#include <stddef.h>

void Chassis_CalculateWheelRpm(
    float vy, float vx, float vw,
    float wheel_rpm[CHASSIS_WHEEL_COUNT])
{
    if (wheel_rpm == NULL)
    {
        return;
    }

    wheel_rpm[0] = vw + vy - vx;
    wheel_rpm[1] = vw + vy + vx;
    wheel_rpm[2] = vw - vy + vx;
    wheel_rpm[3] = vw - vy - vx;
}

int16_t Chassis_EncodeRpmX10(float rpm, bool fine)
{
    float encoded = fine ? rpm * 10.0f : (float)((int16_t)rpm) * 10.0f;

    if (encoded > (float)CHASSIS_MAX_RPM_X10)
    {
        encoded = (float)CHASSIS_MAX_RPM_X10;
    }
    else if (encoded < (float)-CHASSIS_MAX_RPM_X10)
    {
        encoded = (float)-CHASSIS_MAX_RPM_X10;
    }
    return (int16_t)(encoded >= 0.0f ? encoded + 0.5f : encoded - 0.5f);
}
