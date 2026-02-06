//
// Created by lgsell on 15.11.2024.
//
// Handling Modbus RTU communication as a slave device
//
// Key features and functionalities:
// - Management of internal Modbus register maps (Coils, Discrete Inputs, Input Registers, Holding Registers)
// - Processing Modbus read and write operations for different register types
// - Validation and handling of incoming Modbus requests
// - Sending appropriate Modbus responses
// - Ensuring slave device availability and avoiding conflicts during message handling
//

#include "ModbusRTU_Comm.h"
#include "main.h"

// Buffers for receiving and transmitting Modbus messages
volatile uint8_t rx_buffer[MODBUS_MAX_MESSAGE_LENGTH];  // Receive buffer
uint8_t tx_buffer[MODBUS_MAX_MESSAGE_LENGTH]; // Transmit buffer
uint16_t rx_len = 0;	// message + crc
uint16_t message_len = 0;	// message (without crc)
bool slave_busy = 0;	// 0: free, 1: busy

// reference to the UART handler
UART_HandleTypeDef *uart_reference;

/**
 * @brief Receives and processes a Modbus message.
 * @param huart UART handler used for communication
 */
void modbus_receive_message(UART_HandleTypeDef *huart){
    memset(tx_buffer, 0, sizeof(tx_buffer));

    // Store the UART handler in the global variable
    uart_reference = huart;

    // Check if the slave is busy
    if (slave_busy) {
        modbus_error_handler(MODBUS_ERR_SLAVE_BUSY);
        return;
    }

	if(modbus_is_valid_message(rx_len)) { // valid message?
		slave_busy = 1;
		modbus_process_request();   // process request and generate message
		modbus_send_response();     // generate CRC and send modbus response
		slave_busy = 0;
	}
	memset(rx_buffer, 0, sizeof(rx_buffer)); // Pause detected, resetting receive buffer
	rx_len = 0; // resetting receive index
}

/**
 * @brief Sends a Modbus response back to the master.
 */
void modbus_send_response() {

	// calculate CRC for the response
	uint16_t crc = CRC16(tx_buffer, message_len); // Calculate CRC for header and data
	tx_buffer[message_len] = (crc >> 8) & 0xFF;    // CRC High Byte
	tx_buffer[message_len +1] = crc & 0xFF;        // CRC Low Byte
    uint16_t tx_len = message_len + 2;  // Add 2 bytes for CRC

    // send response
	HAL_GPIO_WritePin(RS485_Direction_GPIO_Port, RS485_Direction_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(uart_reference, (uint8_t *)tx_buffer, tx_len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(RS485_Direction_GPIO_Port, RS485_Direction_Pin, GPIO_PIN_RESET);
}
