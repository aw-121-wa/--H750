#ifndef DEVICES_IMU_HWT101_H
#define DEVICES_IMU_HWT101_H

#include <stdint.h>

#include "devices/imu/hwt101/hwt101_parser.h"
#include "main.h"

typedef struct
{
    uint32_t valid_frame_count;
    uint32_t discarded_byte_count;
    uint32_t checksum_error_count;
    uint32_t uart_error_count;
    uint32_t dma_restart_count;
    uint32_t last_uart_error;
} Hwt101Diagnostics;

extern Hwt101Data hwt101_data;
extern uint8_t hwt101_update_flag;

/** 启动 HWT101 的循环 DMA 接收。 */
HAL_StatusTypeDef Hwt101_Init(void);

/** 解析 DMA 环形缓冲区中当前所有完整帧。 */
void Hwt101_Process(void);

/** 返回只读的 IMU 通信诊断信息。 */
const volatile Hwt101Diagnostics *Hwt101_GetDiagnostics(void);

/** 发送将偏航角归零的命令。 */
void Hwt101_ZeroYaw(void);

/** 解锁 HWT101 配置寄存器。 */
void Hwt101_Unlock(void);

/** 设置 HWT101 串口波特率为 115200。 */
void Hwt101_SetBaud115200(void);

/** 设置 HWT101 输出速率为 200 Hz。 */
void Hwt101_SetOutput200Hz(void);

/** 保存当前 HWT101 配置。 */
void Hwt101_SaveSettings(void);

typedef Hwt101Diagnostics IMU_Diagnostics;
/* 旧版 IMU API 别名。 */
#define imu                       hwt101_data
#define mpu_flash                 hwt101_update_flag
#define IMU_Receive_Init          Hwt101_Init
#define IMU_Process               Hwt101_Process
#define IMU_GetDiagnostics        Hwt101_GetDiagnostics
#define Imu_setZero               Hwt101_ZeroYaw
#define Imu_unlock_register       Hwt101_Unlock
#define Imu_setset_baudrate_115200 Hwt101_SetBaud115200
#define Imu_setsave_settings      Hwt101_SaveSettings
#define Imu_set500hz              Hwt101_SetOutput200Hz

#endif
