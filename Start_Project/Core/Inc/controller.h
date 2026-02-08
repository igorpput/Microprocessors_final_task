/*
 * controller.h
 *
 *  Created on: 7 lut 2026
 *      Author: igorp
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"

// holds system state
typedef struct {
    // inputs
    int setpoint_lux;      // target (via encoder 2)
    float current_lux;     // sensor read (bh1750)
    float last_measurement;
    int disturbance_val;   // noise val (encoder 1)


    // internal vars
    float error_sum;       // integral sum

    // output
    int control_signal;    // main led pwm (0-1000)
} SystemState;

// main loop (200ms via TIM3)
void Controller_Step(void);

#endif /* INC_CONTROLLER_H_ */
