#ifndef CONTROL_CHASSIS_CHASSIS_H
#define CONTROL_CHASSIS_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

HAL_StatusTypeDef Chassis_Init(void);
HAL_StatusTypeDef Chassis_SetWheelSpeedX10(const int16_t rpm_x10[4]);
HAL_StatusTypeDef Chassis_SetSpeed(float vy, float vx, float vw, bool fine);

void Motor_setspeed(float vy, float vx, float vw);
void Motor_setspeed_fine(float vy, float vx, float vw);
void Motor_Stop(void);

#endif
