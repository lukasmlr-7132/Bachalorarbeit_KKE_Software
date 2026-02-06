//
// Created by lgsell on 27.11.2024.
//

#ifndef MODBUSRTU_LOGIC_H
#define MODBUSRTU_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include <ModbusRTU_CRC.h>
#include <ModbusRTU_Registers.h>
#include <ModbusRTU_Comm.h>

extern volatile uint8_t rx_buffer[MODBUS_MAX_MESSAGE_LENGTH]; // Receive buffer
extern uint8_t tx_buffer[MODBUS_MAX_MESSAGE_LENGTH]; // Transmit buffer

void modbus_process_request();
bool modbus_is_valid_message(const uint16_t rx_len);

#endif //MODBUSRTU_LOGIC_H
