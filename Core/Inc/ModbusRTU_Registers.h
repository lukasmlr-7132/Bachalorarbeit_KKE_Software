//
// Created by lgsell on 18.11.2024.
//

#ifndef MODBUSRTU_REGISTERS_H
#define MODBUSRTU_REGISTERS_H

#include <stdint.h>

#define MODBUS_MAX_MESSAGE_LENGTH 256    // Modbus request/response max length

#define SOFTWARE_VERSION 0x100 // for example, 0x105 equals V1.05

// 00001-09999: Coils (Function Codes 0x01, 0x05)
typedef enum {
	COIL_OUTPUT_RELEASE, 	// Register 00001, Bit 0
	COIL_ENABLE_OUTPUT_1,	// Register 00001, Bit 1, ON: Enables output 1
	COIL_ENABLE_OUTPUT_2,	// Register 00001, Bit 2, ON: Enables output 2
	COIL_POLARITY_OUTPUT_1,	// Register 00001, Bit 3, ON: Positive output
	COIL_POLARITY_OUTPUT_2,	// Register 00001, Bit 4, ON: Positive output
	// Additional registers can be initialized here
	NUM_COILS
} Modbus_Coils;

// 10001-19999: Discrete Inputs (Function Code 0x02)
typedef enum {
	DISCRETE_HEARTBEAT,				// Register 10001, Bit 0
	DISCRETE_OPERATION_OUTPUT_1,	// Register 10001, Bit 1
	DISCRETE_POLARITY_OUTPUT_1,		// Register 10001, Bit 2
	DISCRETE_WARNING_OUTPUT_1,		// Register 10001, Bit 3
	DISCRETE_FAULT_OUTPUT_1,		// Register 10001, Bit 4
	DISCRETE_OPERATION_OUTPUT_2,	// Register 10001, Bit 5
	DISCRETE_POLARITY_OUTPUT_2,		// Register 10001, Bit 6
	DISCRETE_WARNING_OUTPUT_2,		// Register 10001, Bit 7
	DISCRETE_FAULT_OUTPUT_2,		// Register 10001, Bit 8
	// Additional registers can be initialized here
	NUM_DISCRETE_INPUTS
} Modbus_DiscreteInputs;

// 30001-39999: Input Registers (Function Code 0x04)
typedef enum {
	INPUT_REG_OUTPUT_VOLTAGE_1,  // Register 30001: Output Voltage of Output 1 (mV)
	INPUT_REG_OUTPUT_CURRENT_1,  // Register 30002: Output Current of Output 1 (mA)
	INPUT_REG_OUTPUT_VOLTAGE_2,  // Register 30003: Output Voltage of Output 2 (mV)
	INPUT_REG_OUTPUT_CURRENT_2,  // Register 30004: Output Current of Output 2 (mA)
	INPUT_REG_MCU_TEMPERATURE,   // Register 30005: Temperature of MCU (°C), Factor 0.1
	INPUT_REG_INPUT_VOLTAGE_VCC, // Register 30006: Input Voltage VCC (mV)
	INPUT_REG_VOLTAGE_12V,       // Register 30007: Voltage 12V (mV)
	INPUT_REG_SOFTWARE_VERSION,  // Register 30008: Software Version, Factor 0.01
	// Additional registers can be initialized here
	NUM_ANALOGUE_INPUTS
} Modbus_AnalogueInputs;

// 40001-49999: Holding Registers (Function Codes 0x03, 0x06)
typedef enum {
	HOLDING_REG_SET_VOLTAGE_1,   // Register 40001: Desired Output Voltage 1 (mV)
	HOLDING_REG_SET_CURRENT_1,   // Register 40002: Desired Output Current 1 (mA)
	HOLDING_REG_SLAVE_ADDRESS,   // Register 40005: Slave Address (Stored in non-volatile memory)
	// Additional registers can be initialized here
	NUM_HOLDING_REGISTERS
} Modbus_HoldingRegisters;

extern volatile uint8_t rx_buffer[MODBUS_MAX_MESSAGE_LENGTH]; // Receive buffer
extern uint8_t tx_buffer[MODBUS_MAX_MESSAGE_LENGTH]; // Transmit buffer
extern uint16_t message_len; // message + crc
extern uint16_t slave_addr; // slave address
extern uint16_t function_code; // function code
extern uint16_t start_address; // Start address for operations
extern int16_t val_registers; // Number of registers involved

// Modbus Function Codes
#define MODBUS_READ_COILS                0x01
#define MODBUS_READ_DISCRETE_INPUTS      0x02
#define MODBUS_READ_HOLDING_REGISTERS    0x03
#define MODBUS_READ_INPUT_REGISTERS      0x04
#define MODBUS_WRITE_SINGLE_COIL         0x05
#define MODBUS_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_READ_EXCEPTION_STATUS     0x07
#define MODBUS_DIAGNOSTICS               0x08
#define MODBUS_GET_COMM_EVENT_COUNTER    0x0B
#define MODBUS_GET_COMM_EVENT_LOG        0x0C
#define MODBUS_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_WRITE_MULTIPLE_REGISTERS  0x10
#define MODBUS_REPORT_SLAVE_ID           0x11
#define MODBUS_READ_GENERAL_REFERENCE    0x14
#define MODBUS_WRITE_GENERAL_REFERENCE   0x15
#define MODBUS_MASK_WRITE_REGISTER       0x16
#define MODBUS_READ_WRITE_REGISTERS      0x17

// Error
extern uint8_t exception_status; // exception status
void modbus_error_handler(const uint16_t exception_status);

// Modbus Error Codes
#define MODBUS_ERR_ILLEGAL_FUNCTION           0x01
#define MODBUS_ERR_ILLEGAL_DATA_ADDRESS       0x02
#define MODBUS_ERR_ILLEGAL_DATA_VALUE         0x03
#define MODBUS_ERR_SLAVE_DEVICE_FAILURE       0x04
#define MODBUS_ERR_ACKNOWLEDGE                0x05
#define MODBUS_ERR_SLAVE_BUSY                 0x06
#define MODBUS_ERR_NEGATIVE_ACKNOWLEDGE       0x07
#define MODBUS_ERR_MEMORY_PARITY_ERROR        0x08

void initialize_peripherals();

void modbus_handle_read_coils();
void modbus_handle_read_discrete_inputs();
void modbus_handle_read_holding_registers();
void modbus_handle_read_input_registers();
void modbus_handle_write_single_coil();
void modbus_handle_write_single_register();
void modbus_handle_read_exception_status();
//void modbus_handle_diagnostics();
//void modbus_handle_get_comm_event_counter();
//void modbus_handle_get_comm_event_log();
//void modbus_handle_write_multiple_coils();
//void modbus_handle_write_multiple_registers();

#endif //MODBUSRTU_REGISTERS_H
