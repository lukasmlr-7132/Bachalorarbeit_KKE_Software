//
// Created by lgsell on 18.11.2024.
//
// Functions for handling Modbus register operations.
// The Modbus response is generated here, except for the CRC generation, which is handled in ModbusRTU_Comm.c
//
// Supported Modbus register types:
// - Coils (0x01, 0x05): Binary outputs, readable and writable
// - Discrete Inputs (0x02): Binary inputs, read-only
// - Input Registers (0x04): 16-bit data registers, read-only
// - Holding Registers (0x03, 0x06): 16-bit data registers, readable and writable
//
// Modbus message structure:
// | Slave Addr  | Func Code  | Byte Count |  Data   | CRC     |
// |    1 byte   |    1 byte  |   1 byte   | N bytes | 2 bytes |
//

#include "ModbusRTU_Registers.h"
#include "ModbusRTU_Comm.h"

uint16_t crc = 0;           // Calculated CRC

uint16_t discrete_output_coils[(NUM_COILS/16)+1] = {0};           // Coils (00001-09999)
uint16_t discrete_input_contact[(NUM_DISCRETE_INPUTS/16)+1] = {0}; // Discrete Inputs (10001-19999)
int32_t analogue_input_registers[NUM_ANALOGUE_INPUTS] = {0}; // Input Registers (30001-39999)
int32_t analogue_holding_registers[NUM_HOLDING_REGISTERS] = {0}; // Holding Registers (40001-49999)

/**
 * @brief Initialize Modbus-related peripherals and registers.
 *  */
void initialize_peripherals() {

    // Initialize coils (00001-09999)
    for (int i = 0; i < ((NUM_COILS/16)+1); i++) {
        discrete_output_coils[i] = 0;
    }

    // Initialize holding registers (40001-49999)
    analogue_holding_registers[HOLDING_REG_SET_VOLTAGE_1] = (int16_t)0;  // Desired Output Voltage 1
    analogue_holding_registers[HOLDING_REG_SET_CURRENT_1] = (int16_t)0;  // Desired Output Current 1
    analogue_holding_registers[HOLDING_REG_SLAVE_ADDRESS] = (uint16_t)10; // Slave Address (Stored in non-volatile memory)
}

/**
 * @brief Handle "Read Coils" Modbus function (0x01).
 * @note Generates an error response if the requested address range is invalid.
 */
void modbus_handle_read_coils() {

    // Validate address range
	if (start_address + val_registers > NUM_COILS) {
	    exception_status = MODBUS_ERR_ILLEGAL_DATA_ADDRESS;
	    modbus_error_handler(exception_status);
	    return;
	}

    tx_buffer[0] = slave_addr;  // Slave address
    tx_buffer[1] = function_code;  // Function code
    tx_buffer[2] = (val_registers + 7) / 8; // number of Bytes, to represent the Coils

    // Populate coil states in the response
    for (uint16_t i = 0; i < val_registers; i++) {
    	uint16_t bit_index = start_address + i;
    	uint16_t array_index = bit_index / 16;
        uint16_t bit_position = bit_index % 16;
        if (discrete_output_coils[array_index] & (1 << bit_position)) {
        	tx_buffer[3 + i / 8] |= (1 << (i % 8)); // set bit if input is high
        }
	}

    message_len = 3 + tx_buffer[2];  // Slave address (1 byte) + Function code (1 byte) + Byte count (tx_buffer[2] bytes)

    exception_status = 0;
}

/**
 * @brief Handle "Read Discrete Inputs" Modbus function (0x02).
 * @note Generates an error response if the requested address range is invalid.
 */
void modbus_handle_read_discrete_inputs() {

    // Validate address range
    if (start_address + val_registers > NUM_DISCRETE_INPUTS) {
        exception_status = MODBUS_ERR_ILLEGAL_DATA_ADDRESS;
        modbus_error_handler( exception_status);
        return;
    }

    tx_buffer[0] = slave_addr;  // Slave address
    tx_buffer[1] = function_code;  // Function code
    tx_buffer[2] = (val_registers + 7) / 8; // Number of bytes

    for (uint16_t i = 0; i < val_registers; i++) {
    	uint16_t bit_index = start_address + i;
    	uint16_t array_index = bit_index / 16;
    	uint16_t bit_position = bit_index % 16;
    	if (discrete_input_contact[array_index] & (1 << bit_position)) {
            tx_buffer[3 + i / 8] |= (1 << (i % 8)); // set bit if input is high
        }
    }

    // Calculate message length and data length (including CRC)
    message_len = 3 + tx_buffer[2];  // Slave address + Function code + Byte count + input data

    exception_status = 0;
}

/**
 * @brief Handle "Read Holding Registers" Modbus function (0x03).
 * @note Generates an error response if the requested address range is invalid.
 */
void modbus_handle_read_holding_registers(){

	// Validate address range
	    if (start_address + val_registers > NUM_HOLDING_REGISTERS) {
        exception_status = MODBUS_ERR_ILLEGAL_DATA_ADDRESS;
        modbus_error_handler(exception_status);
        return;
    }

    tx_buffer[0] = slave_addr;  // Slave address
    tx_buffer[1] = function_code;  // Function code
    tx_buffer[2] = val_registers * 2; // Byte count

    // Populate registers in the response
    for (int i = 0; i < val_registers; i++) {
        const uint16_t register_value = analogue_holding_registers[start_address + i];
        tx_buffer[3 + i * 2] = (register_value >> 8) & 0xFF; // High byte
        tx_buffer[4 + i * 2] = register_value & 0xFF; // Low byte
    }

    // Calculate message length (Header + Data)
    message_len = 3 + tx_buffer[2];  // Slave address + Function code + Byte count + register data

    exception_status = 0;
}

/**
 * @brief Handle "Read Input Registers" Modbus function (0x04).
 * @note Generates an error response if the requested address range is invalid or the number of registers is out of bounds.
 */
void modbus_handle_read_input_registers() {

    // Check the validity of the number of registers
    if (start_address + val_registers > NUM_ANALOGUE_INPUTS) {
        exception_status = MODBUS_ERR_ILLEGAL_DATA_ADDRESS;
        modbus_error_handler(exception_status);
        return;
    }

    tx_buffer[0] = slave_addr;  // Slave address
    tx_buffer[1] = function_code;  // Function code
    tx_buffer[2] = val_registers * 2;  // Number of bytes to be sent

    // Set values for input registers
    for (int i = 0; i < val_registers; i++) {
        const uint16_t register_value = analogue_input_registers[start_address + i];
        tx_buffer[3 + i * 2] = (register_value >> 8) & 0xFF;
        tx_buffer[4 + i * 2] = register_value & 0xFF;
    }

    // Calculate message length (Header + Data)
    message_len = 3 + tx_buffer[2];  // Slave address + Function code + Byte count + register data

    exception_status = 0;
}

/**
 * @brief Handle "Write Single Coil" Modbus function (0x05).
 * @note Generates an error response if the coil address is invalid.
 */
void modbus_handle_write_single_coil() {

	// Validate address range
	if (start_address > NUM_COILS) {
        exception_status = MODBUS_ERR_ILLEGAL_DATA_ADDRESS;
        modbus_error_handler(exception_status);
        return;
    }

	// Set or clear the specific bit
	if ((uint16_t)val_registers == 0xFF00) {
	    discrete_output_coils[start_address / 16] |= (1 << (start_address % 16));  // Set the bit
	} else {
	    discrete_output_coils[start_address / 16] &= ~(1 << (start_address % 16)); // Clear the bit
	}

    for (int i = 0; i < 6; i++) {
        tx_buffer[i] = rx_buffer[i];
    }

    // Calculate message length (Header + Data)
    message_len = 6;  // 1 byte Slave address + 1 byte Function code + 2 bytes address + 2 bytes coil value

    exception_status = 0;
}

/**
 * @brief Handle "Write Single Register" Modbus function (0x06).
 * @note Generates an error response if the register address is invalid.
 */
void modbus_handle_write_single_register(){

	// Validate address range
	    if (start_address > NUM_HOLDING_REGISTERS) {
        exception_status = MODBUS_ERR_ILLEGAL_DATA_ADDRESS;
        modbus_error_handler(exception_status);
        return;
    }

    // Write the value to the register
    analogue_holding_registers[start_address] = val_registers;

    // Echo the request back as the response
    for (int i = 0; i < 6; i++) {
        tx_buffer[i] = rx_buffer[i];
    }

    // message length (Header + Data)
    message_len = 6; // 1 byte Slave address + 1 byte Function code + 2 bytes address + 2 bytes coil value

    exception_status = 0;
}

/**
 * @brief Handle "Read Exception Status" Modbus function (0x07).
 */
void modbus_handle_read_exception_status(){
    tx_buffer[0] = slave_addr;  // Slave address
    tx_buffer[1] = function_code;  // Function code
    tx_buffer[2] = exception_status;  // Exception status

    // message length (Header + Data)
    message_len = 3; // 1 byte Slave address + 1 byte Function code + 1 exception code
}


/**
 * @brief Generate a Modbus error response.
 * @param exception_status The Modbus exception code to be included in the response.
 */
void modbus_error_handler(const uint16_t exception_status) {

    tx_buffer[0] = slave_addr;   // Set slave address in tx_buffer
    tx_buffer[1] = function_code | 0x80; // function code + 0x80 for error response
    tx_buffer[2] = exception_status;      // Exception-Code

    // message length (Header + Data)
    message_len = 3; // 1 byte Slave address + 1 byte Function code + 1 exception code

    // Send response via UART (RS485 via UART)
    modbus_send_response();
}
