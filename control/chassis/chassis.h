#ifndef CONTROL_CHASSIS_CHASSIS_H
#define CONTROL_CHASSIS_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/** 初始化 CAN 电机驱动并发送同步停止命令。 */
HAL_StatusTypeDef Chassis_Init(void);

/** 发送四个车轮目标（单位 0.1 RPM）并同步启动。 */
HAL_StatusTypeDef Chassis_SetWheelSpeedX10(const int16_t rpm_x10[4]);

/** 将底盘运动转换为车轮转速并发送。 */
HAL_StatusTypeDef Chassis_SetSpeed(float vy, float vx, float vw, bool fine);

/* 旧版底盘控制封装函数。 */
void Motor_setspeed(float vy, float vx, float vw);
void Motor_setspeed_fine(float vy, float vx, float vw);
void Motor_Stop(void);

#endif
