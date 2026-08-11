#ifndef DEVICES_MOTOR_ZDT_X42S_H
#define DEVICES_MOTOR_ZDT_X42S_H

#include <stdbool.h>
#include <stdint.h>

#include "fdcan.h"

#define ZDT_X42S_MOTOR_COUNT 4U

typedef struct
{
    uint32_t tx_success_count;
    uint32_t tx_failure_count;
    uint32_t rx_frame_count;
    uint32_t bus_error_count;
    uint32_t last_error_code;
    uint32_t last_rx_id;
    uint8_t last_function;
    uint8_t last_status;
    uint32_t motor_reply_count[ZDT_X42S_MOTOR_COUNT];
} ZdtX42sDiagnostics;

HAL_StatusTypeDef ZdtX42s_Init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef ZdtX42s_SetSpeedX10(uint8_t motor_id,
                                     int16_t rpm_x10,
                                     uint8_t acceleration,
                                     bool synchronized);
HAL_StatusTypeDef ZdtX42s_Sync(void);
bool ZdtX42s_HasTxCapacity(uint32_t frame_count);
const volatile ZdtX42sDiagnostics *ZdtX42s_GetDiagnostics(void);

typedef ZdtX42sDiagnostics ZDT_CanDiagnostics;
#define ZDT_X42S_CAN_Init       ZdtX42s_Init
#define ZDT_X42S_SetSpeedX10    ZdtX42s_SetSpeedX10
#define ZDT_X42S_Sync           ZdtX42s_Sync
#define ZDT_X42S_HasTxCapacity  ZdtX42s_HasTxCapacity
#define ZDT_X42S_GetDiagnostics ZdtX42s_GetDiagnostics

#endif
