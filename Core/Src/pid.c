/*
 * pid.c
 *
 *  Created on: Dec 26, 2025
 *      Author: filip
 */


#include "pid.h"

static float clampf(float x, float mn, float mx)
{
    if (x < mn) return mn;
    if (x > mx) return mx;
    return x;
}

void PID_Init(PID_t* pid, float kp, float ki, float kd,
              float out_min, float out_max,
              float i_min, float i_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->i_min   = i_min;
    pid->i_max   = i_max;

    PID_Reset(pid);
}

void PID_Reset(PID_t* pid)
{
    pid->integrator = 0.0f;
    pid->prev_meas  = 0.0f;
    pid->first_run  = true;
}

float PID_Update(PID_t* pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    // error
    float e = setpoint - measurement;

    // P
    float p = pid->kp * e;

    // I (anti-windup przez clamp integratora)
    pid->integrator += pid->ki * e * dt;
    pid->integrator  = clampf(pid->integrator, pid->i_min, pid->i_max);

    // D (derivative on measurement)
    float d = 0.0f;
    if (pid->first_run)
    {
        pid->prev_meas = measurement;
        pid->first_run = false;
        d = 0.0f;
    }
    else
    {
        float meas_dot = (measurement - pid->prev_meas) / dt;
        d = -pid->kd * meas_dot;
        pid->prev_meas = measurement;
    }

    float out = p + pid->integrator + d;
    return clampf(out, pid->out_min, pid->out_max);
}
