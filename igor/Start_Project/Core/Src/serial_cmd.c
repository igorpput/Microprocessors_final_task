/*
 * serial_cmd.c
 *
 *  Created on: 7 lut 2026
 *      Author: igorp
 */


#include "serial_cmd.h"
#include "controller.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern int position;

char rx_buffer[50];
uint8_t rx_index = 0;

volatile uint8_t uart_msg_received = 0;
uint8_t rx_data;




static uint8_t Calculate_Checksum(char* str) {
    uint8_t checksum = 0;
    while (*str) {
        checksum ^= *str; // xor every char
        str++;
    }
    return checksum;
}


void Parse_UART_Command(void) {
    char *semicolon_ptr = strchr(rx_buffer, ';');
    if (semicolon_ptr != NULL) {
        *semicolon_ptr = 0;
        char *checksum_str = semicolon_ptr + 1;
        int received_checksum = atoi(checksum_str);
        int calculated_checksum = Calculate_Checksum(rx_buffer);

        if (received_checksum != calculated_checksum) {
            static char err_msg[64];
            sprintf(err_msg, "CHECKSUM ERROR (Recv:%d != Calc:%d)\r\n", received_checksum, calculated_checksum);
            HAL_UART_Transmit(&huart3, (uint8_t*)err_msg, strlen(err_msg), 100);
            return;
        }
    }

    if (strncmp(rx_buffer, "PRCT", 4) == 0) {
        int value = atoi(&rx_buffer[5]);
        if (value > 100) value = 100;
        if (value < 0) value = 0;


        position = value;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, position * 10);

        char *msg = "OK PRCT";
        uint8_t csum = Calculate_Checksum(msg);
        char resp[32];
        sprintf(resp, "%s;%d\r\n", msg, csum);
        HAL_UART_Transmit(&huart3, (uint8_t*)resp, strlen(resp), 100);
    }

//SETS
    else if (strncmp(rx_buffer, "SET", 3) == 0) {
        int value = atoi(&rx_buffer[4]);
        if (value > 2000) value = 2000;
        if (value < 0) value = 0;

        sys_state.setpoint_lux = value;

        __HAL_TIM_SET_COUNTER(&htim8, value * 2);

        char *msg = "OK SET";
        uint8_t csum = Calculate_Checksum(msg);
        char resp[32];
        sprintf(resp, "%s;%d\r\n", msg, csum);
        HAL_UART_Transmit(&huart3, (uint8_t*)resp, strlen(resp), 100);
    }

//GET INFO
    else if (strncmp(rx_buffer, "GET INFO", 8) == 0) {
        char data_part[64];
        char full_msg[80];

        sprintf(data_part, "INFO %d %d %d", sys_state.setpoint_lux, (int)sys_state.current_lux, position);
        uint8_t csum = Calculate_Checksum(data_part);
        sprintf(full_msg, "%s;%d\r\n", data_part, csum);

        HAL_UART_Transmit(&huart3, (uint8_t*)full_msg, strlen(full_msg), 100);
    }
}


void Serial_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        if (rx_data == '\n' || rx_data == '\r') {
            rx_buffer[rx_index] = 0;
            uart_msg_received = 1;//main flag
            rx_index = 0;
        }
        else {
            if (rx_index < 49) rx_buffer[rx_index++] = rx_data;
        }
        //cont
        HAL_UART_Receive_IT(&huart3, &rx_data, 1);
    }
}
