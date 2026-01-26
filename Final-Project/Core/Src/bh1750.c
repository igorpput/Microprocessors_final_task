/*
 * bh1750.c
 *
 *  Created on: Jan 26, 2026
 *      Author: Abdelrahman Salama
 */
#include "bh1750.h"

#define BH1750_ADDR   (0x23 << 1)
#define POWER_ON      0x01
#define RESET         0x07
#define CONT_HRES     0x10

HAL_StatusTypeDef BH1750_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd;

    cmd = POWER_ON;
    if (HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR, &cmd, 1, 100) != HAL_OK)
        return HAL_ERROR;

    cmd = RESET;
    if (HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR, &cmd, 1, 100) != HAL_OK)
        return HAL_ERROR;

    cmd = CONT_HRES;
    if (HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR, &cmd, 1, 100) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef BH1750_ReadLux(I2C_HandleTypeDef *hi2c, float *lux)
{
    uint8_t data[2];

    if (HAL_I2C_Master_Receive(hi2c, BH1750_ADDR, data, 2, 200) != HAL_OK)
        return HAL_ERROR;

    uint16_t raw = (data[0] << 8) | data[1];
    *lux = raw / 1.2f;

    return HAL_OK;
}


