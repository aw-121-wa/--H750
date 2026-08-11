#ifndef CONTROL_HEADING_HEADING_CONTROL_H
#define CONTROL_HEADING_HEADING_CONTROL_H

#include "control/common/pid.h"
#include "main.h"

typedef struct IMU_RUNDATA
{
    union { float angle; float ANGEL; };
    union { float last_angle; float LAST_ANGEL; };
    union { float speed; float SPEED; };
} HeadingRunState;

extern HeadingRunState heading_run;
extern HeadingRunState heading_turn;
extern PidController heading_pid;

#define HEADING_DRIFT_COMPENSATION_ENABLED 0U

HAL_StatusTypeDef Heading_Init(void);
void Heading_Calibrate(int target_angle);
float Heading_TurnOutput(float target_angle);
float Heading_RelativeAngle(float yaw, float reference_angle);
float Heading_CurrentRelativeAngle(float reference_angle);
float Heading_NormalizeAngle(float angle);
float Heading_CorrectDrift(float expected_current, float target_angle);

typedef HeadingRunState IMU_RUNDATA;
#define Imu_run                   heading_run
#define Imu_turn                  heading_turn
#define Gyro_Pid                  heading_pid
#define YAW_DRIFT_COMP_ENABLE     HEADING_DRIFT_COMPENSATION_ENABLED
#define Gyro_Init                 Heading_Init
#define Direction_Calibration     Heading_Calibrate
#define Direction_Calibration_turn Heading_TurnOutput
#define getAngleZ                 Heading_RelativeAngle
#define getAngleZ_avg             Heading_CurrentRelativeAngle
#define normalize_angle           Heading_NormalizeAngle
#define Yaw_DriftCorrect          Heading_CorrectDrift

#endif
