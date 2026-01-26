/*
 * pi_controller.c
 *
 *  Created on: Jan 26, 2026
 *      Author: Abdelrahman Salama
 */
#include "pi_controller.h"

void PI_Init(PI_t *pi,
             float kp,
             float ki,
             float Ts,
             float out_min,
             float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->Ts = Ts;
    pi->integrator = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
}

float PI_Update(PI_t *pi, float ref, float meas)
{
    float error = ref - meas;

    pi->integrator += pi->ki * pi->Ts * error;

    float out = pi->kp * error + pi->integrator;

    if (out > pi->out_max) out = pi->out_max;
    if (out < pi->out_min) out = pi->out_min;

    return out;
}


