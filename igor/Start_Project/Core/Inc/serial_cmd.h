/*
 * serial_cmd.h
 *
 *  Created on: 7 lut 2026
 *      Author: igorp
 */

#ifndef INC_SERIAL_CMD_H_
#define INC_SERIAL_CMD_H_

#include "main.h"

extern volatile uint8_t uart_msg_received;

extern uint8_t rx_data;

void Parse_UART_Command(void);
void Serial_RxCallback(UART_HandleTypeDef *huart);

#endif /* INC_SERIAL_CMD_H_ */
