/*
 * pwm_led.c
 *
 *  Created on: Jan 26, 2026
 *      Author: Abdelrahman Salama
 */

#include "pwm_led.h"

static float clamp(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

void PWM_LED_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    HAL_TIM_PWM_Start(htim, channel);
}

void PWM_LED_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, float duty)
{
    duty = clamp(duty);

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = (uint32_t)(duty * arr);

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}

