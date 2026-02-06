//
// Created by lgsell on 15.11.2024.
//

#ifndef MODBUSRTU_SLAVE_H
#define MODBUSRTU_SLAVE_H

#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include <ModbusRTU_Logic.h>
#include <ModbusRTU_Registers.h>

extern UART_HandleTypeDef huart4;
extern uint16_t rx_len;

#define BITS_PER_CHAR 10  // 1 start bit + 8 data bits + 1 stop bit

// Function declarations
void modbus_receive_message(UART_HandleTypeDef *huart);
void modbus_send_response();


#endif //MODBUSRTU_SLAVE_H
