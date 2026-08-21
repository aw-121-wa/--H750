#ifndef CONTROL_LOCALIZATION_LOCALIZATION_SERVICE_H
#define CONTROL_LOCALIZATION_LOCALIZATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t motor_valid_mask;
    uint8_t last_missing_mask;
    uint32_t oldest_sample_age_ms;
    uint32_t sample_timeout_count;
    uint32_t incomplete_sample_count;
    uint32_t sample_span_reject_count;
    bool synchronized_sample_valid;
    bool unreasonable_jump;
} LocalizationServiceStatus;

/** Initializes odometry and the four-motor asynchronous position sampler. */
void LocalizationService_Init(void);

/** Starts a four-motor query batch or integrates a complete reply batch. */
void LocalizationService_Tick(uint32_t now_ms, float imu_yaw_deg);

/** Returns motor freshness and odometry sanity information. */
LocalizationServiceStatus LocalizationService_GetStatus(uint32_t now_ms);

#endif
