/*
 * pi_controller.h
 *
 *  Created on: Jan 26, 2026
 *      Author: Abdelrahman Salama
 */

#ifndef INC_PI_CONTROLLER_H_
#define INC_PI_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float Ts;
    float integrator;
    float out_min;
    float out_max;
} PI_t;

void  PI_Init(PI_t *pi, float kp, float ki, float Ts, float out_min, float out_max);
float PI_Update(PI_t *pi, float ref, float meas);
void  PI_Reset(PI_t *pi, float integrator_init);

#ifdef __cplusplus
}
#endif

#endif /* INC_PI_CONTROLLER_H_ */
