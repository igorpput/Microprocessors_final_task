/*
 * bh1750.h
 *
 *  Created on: Jan 26, 2026
 *      Author: Abdelrahman Salama
 */

#ifndef INC_BH1750_H_
#define INC_BH1750_H_

#include "stm32f7xx_hal.h"

#define BH1750_ADDR_DEFAULT (0x23 << 1)

HAL_StatusTypeDef BH1750_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BH1750_ReadLux(I2C_HandleTypeDef *hi2c, float *lux_out);

#endif
