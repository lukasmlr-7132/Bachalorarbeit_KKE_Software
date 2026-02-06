/*
 * modbus_rtu_mcdi_specific.h
 *
 *  Created on: Mar 5, 2025
 *      Author: fscheran
 */

#ifndef CORE_INC_MODBUS_RTU_MCDI_SPECIFIC_H_
#define CORE_INC_MODBUS_RTU_MCDI_SPECIFIC_H_


#include <stdbool.h>
#include <stdint.h>
#include "modbus_rtu.h"

/*
enum MCDI_Coils{
	mcdiOutputRelease,
	mcdiEnableOutput1,
	mcdiEnableOutput2,
	mcdiPolarityOutput1,
	mcdiPolarityOutput2
} mcdiCoils;

enum MCDI_DescreteInputs{
	mcdiHeartbeat,
	mcdiOperationOutput1,
	mcdiPolarityOutput1,
	mcdiWarningOutput1,
	mcdiFaultOutput1,
	mcdiOperationOutput2,
	mcdiPolarityOutput2,
	mcdiWarningOutput2,
	mcdiFaultOutput2,

} mcdiDescreteInputs;

enum MCDI_HoldingRegisters{
	mcdiRequestedOutputVoltageOutput1_2,
	mcdiRequestedOutputCurrentOutput1_2,
	mcdiSlaveAddress
} mcdiHoldingRegisters;

enum MCDI_ReadInputRegister{
	CurrentOutputVoltageOutput1,
	CurrentOutputCurrentOutput1,
	CurrentOutputVoltageOutput2,
	CurrentOutputCurrentOutput2,
	CurrentTemperatureMCU,
	CurrentVoltageMCU,
	CurrentVoltage12V,
	SoftwareVersion
} mcdiReadInputRegister;
*/

//---MCDI modbus request read functions-------------------------------------------------------------------
void ModbusConnection_MCDI_Request0x1_ReadCoils(struct ModbusConnection* modbusConnection);
void ModbusConnection_MCDI_Request0x2_ReadDiscreteInputs(struct ModbusConnection* modbusConnection);
void ModbusConnection_MCDI_Request0x3_ReadHoldingRegisters(struct ModbusConnection* modbusConnection);
void ModbusConnection_MCDI_Request0x4_ReadInputRegisters(struct ModbusConnection* modbusConnection);

//---MCDI modbus get value function code 0x1 functions-----------------------------------------------------
uint8_t ModbusConnection_MCDI_Get0x1_OutputRelease(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x1_EnableOutput1(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x1_EnableOutput2(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x1_PolarityOutput1(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x1_PolarityOutput2(struct ModbusConnection* modbusConnection);

//---MCDI modbus get value function code 0x2 functions-----------------------------------------------------
uint8_t ModbusConnection_MCDI_Get0x2_Heartbeat(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x2_OperationOutput1(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x2_PolarityOutput1(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x2_WarningOutput1(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x2_FaultOutput1(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x2_OperationOutput2(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x2_PolarityOutput2(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x2_WarningOutput2(struct ModbusConnection* modbusConnection);
uint8_t ModbusConnection_MCDI_Get0x2_FaultOutput2(struct ModbusConnection* modbusConnection);

//---MCDI modbus get value function code 0x3 functions-----------------------------------------------------
int16_t ModbusConnection_MCDI_Get0x3_RequestedOutputVoltageOutput1_2(struct ModbusConnection* modbusConnection);
int16_t ModbusConnection_MCDI_Get0x3_RequestedOutputCurrentOutput1_2(struct ModbusConnection* modbusConnection);
uint16_t ModbusConnection_MCDI_Get0x3_SlaveAddress(struct ModbusConnection* modbusConnection);

//---MCDI modbus get value function code 0x4 functions-----------------------------------------------------
int16_t ModbusConnection_MCDI_Get0x4_CurrentOutputVoltageOutput1(struct ModbusConnection* modbusConnection);
int16_t ModbusConnection_MCDI_Get0x4_CurrentOutputCurrentOutput1(struct ModbusConnection* modbusConnection);
int16_t ModbusConnection_MCDI_Get0x4_CurrentOutputVoltageOutput2(struct ModbusConnection* modbusConnection);
int16_t ModbusConnection_MCDI_Get0x4_CurrentOutputCurrentOutput2(struct ModbusConnection* modbusConnection);
int16_t ModbusConnection_MCDI_Get0x4_CurrentTemperatureMCU (struct ModbusConnection* modbusConnection);
uint16_t ModbusConnection_MCDI_Get0x4_CurrentVoltageMCU(struct ModbusConnection* modbusConnection);
uint16_t ModbusConnection_MCDI_Get0x4_CurrentVoltage12V(struct ModbusConnection* modbusConnection);
uint16_t ModbusConnection_MCDI_Get0x4_SoftwareVersion(struct ModbusConnection* modbusConnection);

//---MCDI modbus request write functions-------------------------------------------------------------------
void ModbusConnection_MCDI_WriteRequest_OutputRelease(struct ModbusConnection* modbusConnection, bool newValue);
void ModbusConnection_MCDI_WriteRequest_EnableOutput1(struct ModbusConnection* modbusConnection, bool newValue);
void ModbusConnection_MCDI_WriteRequest_EnableOutput2(struct ModbusConnection* , bool newValue);
void ModbusConnection_MCDI_WriteRequest_PolarityOutput1(struct ModbusConnection* modbusConnection, bool newValue);
void ModbusConnection_MCDI_WriteRequest_PolarityOutput2(struct ModbusConnection* modbusConnection, bool newValue);

void ModbusConnection_MCDI_WriteRequest_RequestedOutputVoltageOutput1_2(struct ModbusConnection* modbusConnection, int16_t newValue);
void ModbusConnection_MCDI_WriteRequest_RequestedOutputCurrentOutput1_2(struct ModbusConnection* modbusConnection, int16_t newValue);
void ModbusConnection_MCDI_WriteRequest_SlaveAddress(struct ModbusConnection* modbusConnection, uint16_t newValue);





#endif /* CORE_INC_MODBUS_RTU_MCDI_SPECIFIC_H_ */
