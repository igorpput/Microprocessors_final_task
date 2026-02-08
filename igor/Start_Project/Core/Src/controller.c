/*
 * controller.c
 *
 *  Created on: 7 lut 2026
 *      Author: igorp
 */

#include "controller.h"
#include <stdio.h>
#include <stdlib.h> // for abs()

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim4;
extern I2C_HandleTypeDef hi2c1;

volatile SystemState sys_state = {
    .setpoint_lux = 100,
    .current_lux = 0.0,
    .error_sum = 0.0,
    .control_signal = 0
};

const float Kp = 0.7f;
const float Ki = 0.5f;

const float DEADBAND = 0.2f;

// filter (20% trust in new data)
const float ALPHA = 0.25f;

// PWM Max
const int PWM_MAX = 1000;

// debug vars
volatile int graph_current_lux = 0;
volatile int graph_setpoint_lux = 0;

void Controller_Step(void) {
    // 1. SENSOR READ + STRONG FILTER
    uint8_t buf[2];
    if (HAL_I2C_Master_Receive(&hi2c1, (0x23 << 1), buf, 2, 10) == HAL_OK) {
        uint16_t raw = (buf[0] << 8) | buf[1];
        float measured_val = raw / 1.2f;

        // EMA smoothing - fixes jumps like 136.5 -> 136.1
        sys_state.current_lux = (ALPHA * measured_val) + ((1.0f - ALPHA) * sys_state.current_lux);
    }

    // 2. ENCODER 2 (ANTI-JUMP PROTECTION 0 -> 2000)
    uint32_t enc2_raw = __HAL_TIM_GET_COUNTER(&htim8);

    // >>> FIX: underflow detection <<<
    // if below 0, unsigned wraps to 65535
    // detect this (60000 is way out of range)
    if (enc2_raw > 60000) {
        enc2_raw = 0;
        __HAL_TIM_SET_COUNTER(&htim8, 0); // hw reset to 0
    }

    // scaling & max limit
    sys_state.setpoint_lux = enc2_raw / 2;

    if (sys_state.setpoint_lux > 2000) {
        sys_state.setpoint_lux = 2000;
        __HAL_TIM_SET_COUNTER(&htim8, 4000); // hw reset to max
    }

    // debug
    graph_current_lux = (int)sys_state.current_lux;
    graph_setpoint_lux = sys_state.setpoint_lux;


    // 3. PI ALGO WITH DEADBEAND
    float error = (float)sys_state.setpoint_lux - sys_state.current_lux;

        // calc abs error
        float abs_error = error > 0 ? error : -error;

    // >>> NEW STABILITY LOGIC <<<
    // if error is small (in target zone), DON'T CHANGE PWM.
    // prevents "breathing" light effect.

        if (abs_error < DEADBAND) {
                // perfect match -> zero error to freeze integral
                error = 0.0f;
                // keep error_sum! needs to hold voltage on diode
            }
    else {
        // outside deadband - act!

        float P = Kp * error;

        sys_state.error_sum += error;

        // Anti-Windup
        float integral_limit = PWM_MAX / Ki;
        if (sys_state.error_sum > integral_limit) sys_state.error_sum = integral_limit;
        if (sys_state.error_sum < -integral_limit) sys_state.error_sum = -integral_limit;

        float I = Ki * sys_state.error_sum;

        float output = P + I;

        int pwm_val = (int)output;
        if (pwm_val > PWM_MAX) pwm_val = PWM_MAX;
        if (pwm_val < 0)       pwm_val = 0;

        sys_state.control_signal = pwm_val;
    }

    // 4. WRITE TO HW
    // always send last known good val
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, sys_state.control_signal);
}
