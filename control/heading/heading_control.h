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

/** 初始化航向状态、PID 控制和 IMU 接收。 */
HAL_StatusTypeDef Heading_Init(void);

/** 转弯完成后更新航向参考。 */
void Heading_Calibrate(int target_angle);

/** 返回朝向目标角度的 PID 输出。 */
float Heading_TurnOutput(float target_angle);

/** Returns turn output using a caller-supplied map-frame heading. */
float Heading_TurnOutputForPose(float target_angle, float current_angle);

/** 返回相对于参考角度的偏航角，范围 -180 到 180。 */
float Heading_RelativeAngle(float yaw, float reference_angle);

/** 返回当前 IMU 相对于参考的偏航角。 */
float Heading_CurrentRelativeAngle(float reference_angle);

/** 将角度归一化到 -180 到 180 范围内。 */
float Heading_NormalizeAngle(float angle);

/** 应用可选的航向漂移补偿。 */
float Heading_CorrectDrift(float expected_current, float target_angle);

typedef HeadingRunState IMU_RUNDATA;
/* 旧版航向 API 别名。 */
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
