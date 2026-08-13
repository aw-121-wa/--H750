#ifndef CONTROL_LOCALIZATION_LOCALIZATION_H
#define CONTROL_LOCALIZATION_LOCALIZATION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float x_mm;
    float y_mm;
    float yaw_deg;
    float vx_mm_s;
    float vy_mm_s;
    float yaw_rate_deg_s;
} NavigationPose;

typedef enum
{
    LOCALIZATION_UNANCHORED = 0,
    LOCALIZATION_ANCHORED = 1
} LocalizationPoseState;

typedef struct
{
    LocalizationPoseState pose_state;
    uint32_t last_update_ms;
    bool wheel_data_valid;
    bool imu_data_valid;
} LocalizationStatus;

typedef struct
{
    float longitudinal_mm_per_x10_deg;
    float lateral_mm_per_x10_deg;
    float yaw_scale;
} LocalizationCalibration;

/** Resets relative odometry and marks the map pose as unanchored. */
void Localization_Init(void);

/** Integrates one synchronized four-wheel sample and an IMU yaw sample. */
void Localization_Update(const int32_t wheel_position_x10_deg[4],
                         float imu_yaw_deg,
                         uint32_t now_ms);

/** Anchors the current odometry location to a map pose without losing baselines. */
bool Localization_SetMapPose(const NavigationPose *pose);

/** Applies validated longitudinal, lateral and yaw calibration factors. */
bool Localization_SetCalibration(const LocalizationCalibration *calibration);

/** Returns the latest pose in millimetres and degrees. */
NavigationPose Localization_GetPose(void);

/** Returns validity, anchoring and update timing information. */
LocalizationStatus Localization_GetStatus(void);

#endif
