/*
 * bh1750.c
 *
 *  Created on: Jan 26, 2026
 *      Author: Abdelrahman Salama
 */
#include "bh1750.h"

static HAL_StatusTypeDef bh1750_cmd(I2C_HandleTypeDef *hi2c, uint8_t cmd)
{
    return HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR_DEFAULT, &cmd, 1, 100);
}

HAL_StatusTypeDef BH1750_Init(I2C_HandleTypeDef *hi2c)
{
    // Power on
    if (bh1750_cmd(hi2c, 0x01) != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);

    // Reset (optional, needs power on)
    if (bh1750_cmd(hi2c, 0x07) != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);

    // Continuous H-Resolution mode (1 lx resolution)
    if (bh1750_cmd(hi2c, 0x10) != HAL_OK) return HAL_ERROR;

    HAL_Delay(180); // first measurement ready
    return HAL_OK;
}

HAL_StatusTypeDef BH1750_ReadLux(I2C_HandleTypeDef *hi2c, float *lux_out)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(hi2c, BH1750_ADDR_DEFAULT, buf, 2, 100);
    if (st != HAL_OK) return st;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    // datasheet: lux = raw / 1.2
    *lux_out = (float)raw / 1.2f;
    return HAL_OK;
}
