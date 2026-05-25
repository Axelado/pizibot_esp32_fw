#include "pid.h"
#include <math.h>

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp       = kp;
    pid->ki       = ki;
    pid->kd       = kd;
    pid->out_min  = out_min;
    pid->out_max  = out_max;
    pid->integral     = 0.0f;
    pid->prev_measured = 0.0f;
}

float pid_compute(pid_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    /* Integral with anti-windup: clamp to ±(out_max/ki) so the integral
     * contribution alone never exceeds the output limit.
     * Also guards against NaN corruption (NaN comparisons always fail). */
    pid->integral += error * dt;
    if (!isfinite(pid->integral)) pid->integral = 0.0f;
    float int_limit = (pid->ki > 0.0f) ? (pid->out_max / pid->ki) : pid->out_max;
    if (pid->integral >  int_limit) pid->integral =  int_limit;
    if (pid->integral < -int_limit) pid->integral = -int_limit;

    /* Derivative on measurement — avoids derivative kick on setpoint changes */
    float derivative = -(measured - pid->prev_measured) / dt;
    pid->prev_measured = measured;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    /* Output saturation */
    if (output >  pid->out_max) output =  pid->out_max;
    if (output < -pid->out_max) output = -pid->out_max;

    return output;
}

void pid_reset(pid_t *pid)
{
    pid->integral      = 0.0f;
    pid->prev_measured = 0.0f;
}
