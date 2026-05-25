/*
 * pid.h
 *
 *  Created on: Dec 26, 2025
 *      Author: filip
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integrator;
    float prev_meas;
    bool  first_run;

    float out_min;
    float out_max;

    float i_min;
    float i_max;
} PID_t;

void  PID_Init(PID_t* pid, float kp, float ki, float kd,
               float out_min, float out_max,
               float i_min, float i_max);

void  PID_Reset(PID_t* pid);

/**
 * PID update (derivative on measurement -> mniej szumu)
 * setpoint: wartość zadana (deg)
 * measurement: zmierzona (deg)
 * dt: czas w sekundach
 */
float PID_Update(PID_t* pid, float setpoint, float measurement, float dt);


#endif /* INC_PID_H_ */
