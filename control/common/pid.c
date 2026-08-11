#include "control/common/pid.h"

#include <stddef.h>

static float Pid_Abs(float value)
{
    return value < 0.0f ? -value : value;
}

void Pid_Init(PidController *pid, float kp, float ki, float kd,
              float output_max, float output_min)
{
    if (pid == NULL)
    {
        return;
    }

    *pid = (PidController){
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .integral_max = 100.0f,
        .output_min = output_min,
        .output_max = output_max,
    };
}

float Pid_Compute(PidController *pid, float setpoint, float measurement)
{
    float error;
    float output;

    if (pid == NULL)
    {
        return 0.0f;
    }

    error = setpoint - measurement;
    pid->integral += error;
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < -pid->integral_max)
    {
        pid->integral = -pid->integral_max;
    }

    output = pid->kp * error + pid->ki * pid->integral +
             pid->kd * (error - pid->previous_error);
    pid->previous_error = error;

    if (output > pid->output_max)
    {
        return pid->output_max;
    }
    if (output < pid->output_min)
    {
        return pid->output_min;
    }
    return output;
}

void PidSeparated_Init(PidSeparatedController *pid,
                       float kp, float ki, float kd,
                       float output_max, float output_min,
                       float threshold)
{
    if (pid == NULL)
    {
        return;
    }
    Pid_Init(&pid->base, kp, ki, kd, output_max, output_min);
    pid->threshold = threshold;
}

float PidSeparated_Compute(PidSeparatedController *pid,
                           float setpoint, float measurement)
{
    float saved_ki;
    float output;

    if (pid == NULL)
    {
        return 0.0f;
    }

    saved_ki = pid->base.ki;
    if (Pid_Abs(setpoint - measurement) > pid->threshold)
    {
        pid->base.ki = 0.0f;
    }
    output = Pid_Compute(&pid->base, setpoint, measurement);
    pid->base.ki = saved_ki;
    return output;
}
