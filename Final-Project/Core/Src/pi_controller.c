/*
 * pi_controller.c
 *
 *  Created on: Jan 26, 2026
 *      Author: Abdelrahman Salama
 */
#include "pi_controller.h"

void PI_Init(PI_t *pi, float kp, float ki, float Ts, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->Ts = Ts;
    pi->integrator = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
}

void PI_Reset(PI_t *pi, float integrator_init)
{
    pi->integrator = integrator_init;
}

float PI_Update(PI_t *pi, float ref, float meas)
{
    float e = ref - meas;

    // PI
    float p = pi->kp * e;
    float i = pi->integrator + (pi->ki * pi->Ts * e);
    float u = p + i;

    // Saturate
    if (u > pi->out_max) u = pi->out_max;
    if (u < pi->out_min) u = pi->out_min;

    // Anti-windup (clamp integrator so that p+i == u after saturation)
    pi->integrator = u - p;

    return u;
}



