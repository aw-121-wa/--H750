#include "control/localization/localization.h"

#include <math.h>
#include <string.h>

#define LOCALIZATION_PI 3.14159265358979323846f
#define LOCALIZATION_DEFAULT_MM_PER_X10_DEG 0.06723f

static NavigationPose s_pose;
static LocalizationStatus s_status;
static LocalizationCalibration s_calibration;
static int32_t s_last_wheel[4];
static float s_last_yaw;
static float s_last_imu_yaw;
static float s_map_yaw_offset;
static bool s_started;

static float NormalizeAngle(float angle)
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

void Localization_Init(void)
{
    memset(&s_pose, 0, sizeof(s_pose));
    memset(&s_status, 0, sizeof(s_status));
    memset(s_last_wheel, 0, sizeof(s_last_wheel));
    s_calibration.longitudinal_mm_per_x10_deg =
        LOCALIZATION_DEFAULT_MM_PER_X10_DEG;
    s_calibration.lateral_mm_per_x10_deg =
        LOCALIZATION_DEFAULT_MM_PER_X10_DEG;
    s_calibration.yaw_scale = 1.0f;
    s_status.pose_state = LOCALIZATION_UNANCHORED;
    s_started = false;
    s_last_yaw = 0.0f;
    s_last_imu_yaw = 0.0f;
    s_map_yaw_offset = 0.0f;
}

void Localization_Update(const int32_t wheel_position_x10_deg[4],
                         float imu_yaw_deg,
                         uint32_t now_ms)
{
    int32_t delta[4];
    float body_forward_mm;
    float body_left_mm;
    float yaw_deg;
    float yaw_mid_rad;
    float dt_s;
    float world_dx;
    float world_dy;
    float delta_yaw;

    if (wheel_position_x10_deg == NULL || !isfinite(imu_yaw_deg))
    {
        return;
    }
    if (!s_started)
    {
        memcpy(s_last_wheel, wheel_position_x10_deg, sizeof(s_last_wheel));
        s_last_imu_yaw = imu_yaw_deg;
        s_map_yaw_offset = NormalizeAngle(-imu_yaw_deg *
                                          s_calibration.yaw_scale);
        s_pose.yaw_deg = NormalizeAngle(imu_yaw_deg * s_calibration.yaw_scale +
                                        s_map_yaw_offset);
        s_last_yaw = s_pose.yaw_deg;
        s_status.last_update_ms = now_ms;
        s_status.wheel_data_valid = true;
        s_status.imu_data_valid = true;
        s_started = true;
        return;
    }

    for (uint32_t i = 0U; i < 4U; ++i)
    {
        delta[i] = wheel_position_x10_deg[i] - s_last_wheel[i];
        s_last_wheel[i] = wheel_position_x10_deg[i];
    }
    body_forward_mm = ((float)delta[0] + (float)delta[1] -
                       (float)delta[2] - (float)delta[3]) * 0.25f *
                      s_calibration.longitudinal_mm_per_x10_deg;
    body_left_mm = (-(float)delta[0] + (float)delta[1] +
                    (float)delta[2] - (float)delta[3]) * 0.25f *
                   s_calibration.lateral_mm_per_x10_deg;

    s_last_imu_yaw = imu_yaw_deg;
    yaw_deg = NormalizeAngle(imu_yaw_deg * s_calibration.yaw_scale +
                             s_map_yaw_offset);
    delta_yaw = NormalizeAngle(yaw_deg - s_last_yaw);
    yaw_mid_rad = NormalizeAngle(s_last_yaw + delta_yaw * 0.5f) *
                  LOCALIZATION_PI / 180.0f;
    world_dx = body_forward_mm * cosf(yaw_mid_rad) -
               body_left_mm * sinf(yaw_mid_rad);
    world_dy = body_forward_mm * sinf(yaw_mid_rad) +
               body_left_mm * cosf(yaw_mid_rad);

    dt_s = (float)(now_ms - s_status.last_update_ms) * 0.001f;
    s_pose.x_mm += world_dx;
    s_pose.y_mm += world_dy;
    s_pose.yaw_deg = yaw_deg;
    if (dt_s > 0.0f)
    {
        s_pose.vx_mm_s = world_dx / dt_s;
        s_pose.vy_mm_s = world_dy / dt_s;
        s_pose.yaw_rate_deg_s = delta_yaw / dt_s;
    }
    s_last_yaw = yaw_deg;
    s_status.last_update_ms = now_ms;
    s_status.wheel_data_valid = true;
    s_status.imu_data_valid = true;
}

bool Localization_SetMapPose(const NavigationPose *pose)
{
    if (pose == NULL || !isfinite(pose->x_mm) || !isfinite(pose->y_mm) ||
        !isfinite(pose->yaw_deg))
    {
        return false;
    }
    s_map_yaw_offset = NormalizeAngle(pose->yaw_deg -
                                      s_last_imu_yaw * s_calibration.yaw_scale);
    s_pose = *pose;
    s_pose.yaw_deg = NormalizeAngle(s_pose.yaw_deg);
    s_last_yaw = s_pose.yaw_deg;
    s_status.pose_state = LOCALIZATION_ANCHORED;
    return true;
}

bool Localization_SetCalibration(const LocalizationCalibration *calibration)
{
    if (calibration == NULL ||
        calibration->longitudinal_mm_per_x10_deg <= 0.0f ||
        calibration->lateral_mm_per_x10_deg <= 0.0f ||
        calibration->yaw_scale <= 0.0f ||
        !isfinite(calibration->longitudinal_mm_per_x10_deg) ||
        !isfinite(calibration->lateral_mm_per_x10_deg) ||
        !isfinite(calibration->yaw_scale))
    {
        return false;
    }
    s_map_yaw_offset = NormalizeAngle(s_pose.yaw_deg -
                                      s_last_imu_yaw * calibration->yaw_scale);
    s_calibration = *calibration;
    return true;
}

NavigationPose Localization_GetPose(void)
{
    return s_pose;
}

LocalizationStatus Localization_GetStatus(void)
{
    return s_status;
}
