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

void Pid_Init(PidController *pid, float kp, float ki, float kd,
              float output_max, float output_min);
float Pid_Compute(PidController *pid, float setpoint, float measurement);
void PidSeparated_Init(PidSeparatedController *pid,
                       float kp, float ki, float kd,
                       float output_max, float output_min,
                       float threshold);
float PidSeparated_Compute(PidSeparatedController *pid,
                           float setpoint, float measurement);

typedef PidController PIDstruct;
typedef PidSeparatedController PIDstructIntegralSeparation;
#define PID_Init                        Pid_Init
#define PID_Compute                     Pid_Compute
#define PID_Init_Integral_separation    PidSeparated_Init
#define PID_Compute_Integral_separation PidSeparated_Compute

#endif
