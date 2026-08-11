#ifndef CONTROL_COMMON_PID_H
#define CONTROL_COMMON_PID_H

typedef struct PIDstruct
{
    float kp;
    float ki;
    float kd;
    union { float integral_max; float integralMax; };
    union { float output_min; float outputMin; };
    union { float output_max; float outputMax; };
    union { float previous_error; float previousError; };
    float integral;
} PidController;

typedef struct PIDstructIntegralSeparation
{
    union { PidController base; PidController PID_basic; };
    union { float threshold; float IntegralSeparationThreshold; };
} PidSeparatedController;

/** 初始化 PID 增益、限幅和运行时状态。 */
void Pid_Init(PidController *pid, float kp, float ki, float kd,
              float output_max, float output_min);

/** 根据设定值和测量值计算一次 PID 输出。 */
float Pid_Compute(PidController *pid, float setpoint, float measurement);

/** 初始化带积分分离的 PID 控制器。 */
void PidSeparated_Init(PidSeparatedController *pid,
                       float kp, float ki, float kd,
                       float output_max, float output_min,
                       float threshold);

/** 计算 PID 输出，在阈值外禁用积分。 */
float PidSeparated_Compute(PidSeparatedController *pid,
                           float setpoint, float measurement);

typedef PidController PIDstruct;
typedef PidSeparatedController PIDstructIntegralSeparation;
/* 旧版 PID API 别名。 */
#define PID_Init                        Pid_Init
#define PID_Compute                     Pid_Compute
#define PID_Init_Integral_separation    PidSeparated_Init
#define PID_Compute_Integral_separation PidSeparated_Compute

#endif
