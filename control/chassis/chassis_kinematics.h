#ifndef CONTROL_CHASSIS_CHASSIS_KINEMATICS_H
#define CONTROL_CHASSIS_CHASSIS_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_WHEEL_COUNT 4U
#define CHASSIS_MAX_RPM_X10 3000

/** 根据底盘运动命令计算四个车轮的转速值。 */
void Chassis_CalculateWheelRpm(float vy,
                               float vx,
                               float vw,
                               float wheel_rpm[CHASSIS_WHEEL_COUNT]);

/** Scales all wheel speeds together when any wheel exceeds the RPM limit. */
void Chassis_NormalizeWheelRpm(float wheel_rpm[CHASSIS_WHEEL_COUNT]);

/** 限幅转速并转换为有符号 0.1 RPM 单位。 */
int16_t Chassis_EncodeRpmX10(float rpm, bool fine);

#endif
