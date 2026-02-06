//
// Created by lgsell on 27.11.2024.
//
// Core logic for processing and validating Modbus requests and responses.
//
// Key functionalities:
// - Validation of received Modbus messages (using CRC checks and verifying message structure)
// - Processing of Modbus requests based on function codes (e.g., reading/writing coils or registers)
// - Handling Modbus exceptions and generating appropriate error responses
// - Managing the Modbus state machine and executing commands from the Modbus master
//

#include "ModbusRTU_Logic.h"

uint16_t slave_addr = 0; // slave address
uint16_t function_code = 0; // function code
uint16_t start_address = 0; // Start address for operations
int16_t val_registers = 0; // Number of registers (read) / register value (write)
uint8_t exception_status = 0; // exception status

/**
 * @brief Processes received Modbus messages.
 * @note Generates an error response if the message is invalid.
 */
void modbus_process_request(){

    // Call the appropriate function handler
    switch (function_code) {
            case MODBUS_READ_COILS:  // 0x01 Read Coils
                modbus_handle_read_coils();
            break;
            case MODBUS_READ_DISCRETE_INPUTS:  // 0x02 Read Discrete Inputs
                modbus_handle_read_discrete_inputs();
            break;
            case MODBUS_READ_HOLDING_REGISTERS:  // 0x03 Read Holding Registers
                modbus_handle_read_holding_registers();
            break;
            case MODBUS_READ_INPUT_REGISTERS:  // 0x04 Read Input Registers
                modbus_handle_read_input_registers();
            break;
            case MODBUS_WRITE_SINGLE_COIL:  // 0x05 Write Single Coil
                modbus_handle_write_single_coil();
            break;
            case MODBUS_WRITE_SINGLE_REGISTER:  // 0x06 Write Single Register
                modbus_handle_write_single_register();
            break;
            case MODBUS_READ_EXCEPTION_STATUS:  // 0x07 Read Exception Status (Serial Line only)
                modbus_handle_read_exception_status();
            break;
//          case MODBUS_DIAGNOSTICS:  // 0x08 Diagnostics (Serial Line only)
//              modbus_handle_diagnostics();
//          break;
//          case MODBUS_GET_COMM_EVENT_COUNTER:  // 0x0B Get Comm Event Counter (Serial Line only)
//              modbus_handle_get_comm_event_counter();
//          break;
//          case MODBUS_GET_COMM_EVENT_LOG:  // 0x0C Get Comm Event Log (Serial Line only)
//              modbus_handle_get_comm_event_log();
//          break;
//          case MODBUS_WRITE_MULTIPLE_COILS:  // 0x0F Write Multiple Coils
//              modbus_handle_write_multiple_coils();
//          break;
//          case MODBUS_WRITE_MULTIPLE_REGISTERS:  // 0x10 Write Multiple Registers
//              modbus_handle_write_multiple_registers();
//          break;

            default: // Invalid function code, send error response
            	exception_status = MODBUS_ERR_ILLEGAL_FUNCTION;
                modbus_error_handler(exception_status);
            break;
    }
}


/**
 * @brief Validates the structure and CRC of a Modbus message.
 * @return 1 if the message is valid, 0 otherwise.
 */
bool modbus_is_valid_message(const uint16_t rx_len) {

	slave_addr = rx_buffer[0];
	function_code = rx_buffer[1];
	start_address = (rx_buffer[2] << 8) | rx_buffer[3];
	val_registers = (rx_buffer[4] << 8) | rx_buffer[5];

    // Validate the range of the requested registers if its a read-function
	if (function_code != MODBUS_WRITE_SINGLE_COIL && function_code != MODBUS_WRITE_SINGLE_REGISTER){
		if ((start_address + val_registers) > 9999){
			exception_status = MODBUS_ERR_ILLEGAL_DATA_ADDRESS;
			modbus_error_handler(exception_status);
			return false;
		}
	}

    // Validate message length
    if (rx_len < 5 || rx_len > MODBUS_MAX_MESSAGE_LENGTH) {
    	exception_status = MODBUS_ERR_ILLEGAL_DATA_VALUE;
        modbus_error_handler(exception_status);
        return false; // Minimum length for Modbus message not met
    }

    // Verify CRC
    const uint16_t crc_calculated = CRC16(rx_buffer, rx_len - 2);  // CRC from rx_buffer without CRC bytes
    const uint16_t crc_received = (rx_buffer[rx_len - 2] << 8) | rx_buffer[rx_len - 1];
    return (crc_calculated == crc_received);
}

