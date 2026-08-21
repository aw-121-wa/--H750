#ifndef DEVICES_MOTOR_ZDT_X42S_H
#define DEVICES_MOTOR_ZDT_X42S_H

#include <stdbool.h>
#include <stdint.h>

#include "fdcan.h"

#define ZDT_X42S_MOTOR_COUNT 4U

typedef struct
{
    int32_t position_x10_deg;
    uint32_t timestamp_ms;
    uint32_t sequence;
    bool valid;
} ZdtX42sPositionSample;

typedef struct
{
    /* 触发：HAL_FDCAN_AddMessageToTxFifoQ 返回 HAL_OK 时 +1 */
    uint32_t tx_success_count;
    /* 触发：Send 入参为 NULL 或 HAL 发送失败时 +1 */
    uint32_t tx_failure_count;
    /* 触发：RxFifo0 回调中 HAL_FDCAN_GetRxMessage 成功时 +1 */
    uint32_t rx_frame_count;
    /* 触发：FDCAN 控制器错误中断 / 总线警告/被动/关闭中断 / GetRxMessage 失败时 +1 */
    uint32_t bus_error_count;
    /* 触发：ErrorCallback 记录 hfdcan->ErrorCode；ErrorStatusCallback 记录 error_status_its */
    uint32_t last_error_code;
    /* 触发：RxFifo0 回调中记录 header.Identifier */
    uint32_t last_rx_id;
    /* 触发：RxFifo0 回调中记录 data[0]（协议功能码） */
    uint8_t last_function;
    /* 触发：RxFifo0 回调中记录 data[1]（电机状态字节） */
    uint8_t last_status;
    /* 触发：RxFifo0 回调中按扩展帧 ID 高字节解析电机 ID（1-4），对应索引 +1 */
    uint32_t motor_reply_count[ZDT_X42S_MOTOR_COUNT];
} ZdtX42sDiagnostics;

/** 配置滤波器，启动 FDCAN，并使能诊断回调。 */
HAL_StatusTypeDef ZdtX42s_Init(FDCAN_HandleTypeDef *hfdcan);

/** 入队一个电机速度命令，单位为有符号 0.1 RPM。 */
HAL_StatusTypeDef ZdtX42s_SetSpeedX10(uint8_t motor_id,
                                     int16_t rpm_x10,
                                     uint16_t acceleration_rpm_s,
                                     bool synchronized);

/** Requests the motor's signed real-time position in 0.1 degree units. */
HAL_StatusTypeDef ZdtX42s_RequestPosition(uint8_t motor_id);

/** Returns the latest position sample and its age in milliseconds. */
bool ZdtX42s_GetPosition(uint8_t motor_id,
                         int32_t *position_x10_deg,
                         uint32_t *age_ms);

/** Copies the latest position reply with its receive sequence. */
bool ZdtX42s_GetPositionSample(uint8_t motor_id,
                               ZdtX42sPositionSample *sample);

/** 入队广播同步启动帧。 */
HAL_StatusTypeDef ZdtX42s_Sync(void);

/** 检查发送 FIFO 是否能容纳请求数量的帧。 */
bool ZdtX42s_HasTxCapacity(uint32_t frame_count);

/** 返回只读的 CAN 通信诊断信息。 */
const volatile ZdtX42sDiagnostics *ZdtX42s_GetDiagnostics(void);

#endif
