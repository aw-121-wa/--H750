#ifndef CONTROL_CHASSIS_CHASSIS_KINEMATICS_H
#define CONTROL_CHASSIS_CHASSIS_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_WHEEL_COUNT 4U
#define CHASSIS_MAX_RPM_X10 3000

void Chassis_CalculateWheelRpm(float vy,
                               float vx,
                               float vw,
                               float wheel_rpm[CHASSIS_WHEEL_COUNT]);
int16_t Chassis_EncodeRpmX10(float rpm, bool fine);

#endif
