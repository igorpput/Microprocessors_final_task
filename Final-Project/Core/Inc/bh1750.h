/*
 * bh1750.h
 *
 *  Created on: Jan 26, 2026
 *      Author: Abdelrahman Salama
 */

#ifndef BH1750_H
#define BH1750_H

#include "stm32f7xx_hal.h"

// initialize sensor
HAL_StatusTypeDef BH1750_Init(I2C_HandleTypeDef *hi2c);

// read lux value
HAL_StatusTypeDef BH1750_ReadLux(I2C_HandleTypeDef *hi2c, float *lux);

#endif

