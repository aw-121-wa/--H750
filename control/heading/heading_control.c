#include "control/heading/heading_control.h"

#include "control/chassis/chassis.h"
#include "devices/imu/hwt101/hwt101.h"

#define HEADING_KP          2.1f
#define HEADING_KI          0.0f
#define HEADING_KD          0.5f
#define HEADING_OUTPUT_MAX  120.0f
#define HEADING_OUTPUT_MIN -120.0f
#define HEADING_TOLERANCE   0.6f

HeadingRunState heading_run;
HeadingRunState heading_turn;
PidController heading_pid;

static float Heading_Abs(float value)
{
    return value < 0.0f ? -value : value;
}

HAL_StatusTypeDef Heading_Init(void)
{
    Pid_Init(&heading_pid, HEADING_KP, HEADING_KI, HEADING_KD,
             HEADING_OUTPUT_MAX, HEADING_OUTPUT_MIN);
    return Hwt101_Init();
}

float Heading_NormalizeAngle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

float Heading_RelativeAngle(float yaw, float reference_angle)
{
    return Heading_NormalizeAngle(yaw - reference_angle);
}

float Heading_CurrentRelativeAngle(float reference_angle)
{
    return Heading_RelativeAngle(hwt101_data.yaw, reference_angle);
}

float Heading_TurnOutput(float target_angle)
{
    const float current = Heading_NormalizeAngle(hwt101_data.yaw);

    return Heading_TurnOutputForPose(target_angle, current);
}

float Heading_TurnOutputForPose(float target_angle, float current_angle)
{
    const float current = Heading_NormalizeAngle(current_angle);
    const float nearest_target = current +
        Heading_NormalizeAngle(target_angle - current);

    return -Pid_Compute(&heading_pid, nearest_target, current);
}

void Heading_Calibrate(int target_angle)
{
    const float error = Heading_NormalizeAngle(
        (float)target_angle - hwt101_data.yaw);

    if (Heading_Abs(error) <= HEADING_TOLERANCE)
    {
        (void)Chassis_Stop();
        return;
    }
    (void)Chassis_SetSpeed(0.0f, 0.0f,
                           Heading_TurnOutput((float)target_angle), true);
}

float Heading_CorrectDrift(float expected_current, float target_angle)
{
#if HEADING_DRIFT_COMPENSATION_ENABLED
    const float error = Heading_NormalizeAngle(
        hwt101_data.yaw - Heading_NormalizeAngle(expected_current));
    return target_angle + error;
#else
    (void)expected_current;
    return target_angle;
#endif
}
