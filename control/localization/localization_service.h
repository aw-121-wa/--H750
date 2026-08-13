#ifndef CONTROL_LOCALIZATION_LOCALIZATION_SERVICE_H
#define CONTROL_LOCALIZATION_LOCALIZATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t motor_valid_mask;
    uint32_t oldest_sample_age_ms;
    bool synchronized_sample_valid;
    bool unreasonable_jump;
} LocalizationServiceStatus;

/** Initializes odometry and the sequential four-motor position poller. */
void LocalizationService_Init(void);

/** Polls one motor at a time and integrates each complete four-wheel sample. */
void LocalizationService_Tick(uint32_t now_ms, float imu_yaw_deg);

/** Returns motor freshness and odometry sanity information. */
LocalizationServiceStatus LocalizationService_GetStatus(uint32_t now_ms);

#endif
