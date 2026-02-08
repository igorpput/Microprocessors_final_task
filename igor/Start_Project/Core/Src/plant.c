/*
 * plant.c
 *
 *  Created on: 7 lut 2026
 *      Author: igorp
 */

#include "plant.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

int position = 0;
static uint32_t last_counter_value = 32000;

void Plant_ProcessDisturbance(void) {
    uint32_t current_counter = __HAL_TIM_GET_COUNTER(&htim1);
    int16_t diff = (int16_t)(current_counter - last_counter_value);

    if (diff != 0) {
        if(diff > 0) position += 5;
        else position -= 5;

        if (position > 100) position = 100;
        if (position < 0) position = 0;

        last_counter_value = current_counter;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, position * 10);
}
