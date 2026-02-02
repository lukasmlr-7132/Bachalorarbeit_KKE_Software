#ifndef INC_STEPPER_H
#define INC_STEPPER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"   // GPIO_TypeDef, HAL_GPIO_WritePin, GPIO_PIN_*

typedef enum {
	GB_OK = 0, GB_ERR_NULL, GB_ERR_INVALID, GB_ERR_RANGE, GB_ERR_BUSY
} GBStatus;

typedef enum {
	STEPPER_OFF = 0, STEPPER_TARGET_OPERATION, STEPPER_REFERENCING
} StepperMode;

typedef struct {
	GPIO_TypeDef *A_port;
	uint16_t A_pin;
	GPIO_TypeDef *B_port;
	uint16_t B_pin;
	GPIO_TypeDef *C_port;
	uint16_t C_pin;
	GPIO_TypeDef *D_port;
	uint16_t D_pin;
} StepperPins;

typedef struct {
	//config
	uint16_t drive_frequency_high_hz;
	uint16_t drive_frequency_low_hz;
	uint16_t drive_psc;
	uint16_t base_frequency;
	uint8_t start_sequence;

	//state
	StepperMode mode;
	uint8_t step_idx;
	uint32_t step_counter;

	//for position operation
	uint32_t target_position;
	uint32_t target_positions[6];

	// current tracking
	uint32_t current_recent;
	uint32_t current_last;
	uint32_t current_diff;
	uint32_t current_sum;
	uint32_t stall_limit;

	// hardware binding
	StepperPins pins;

	// fullstep pattern (4 steps)
	uint8_t step_pattern[4];
} stepper;

// Public API (Funktionsnamen wie in deinem Grundgerüst)
GBStatus stepper_init_hardware(stepper *self, const StepperPins *pins); // NULL => Default-Pattern

GBStatus stepper_init_parameters(stepper *self, uint16_t freq_high_hz,
		uint16_t freq_low_hz, uint16_t base_frequency, uint32_t stall_limit); // wird gesetzt, aber nicht genutzt

GBStatus stepper_init_positions(stepper *self,
		const uint32_t target_positions[6]);

GBStatus stepper_set_target(stepper *self, size_t index);
GBStatus stepper_set_drivefreq_low(stepper *self);
GBStatus stepper_set_drivefreq_high(stepper *self);
GBStatus stepper_enable(stepper *self);
GBStatus stepper_disable(stepper *self);

GBStatus stepper_reference(stepper *self);

// optional: wird zyklisch aus Timer-ISR oder Task gerufen
void stepper_tick(stepper *self);
GBStatus stepper_step_cw(stepper *self);
GBStatus stepper_step_ccw(stepper *self);

void stepper_sound(stepper *self);

//RS485-MODBUSRTU--------------------------------------------------------------------------------------------------

#define MB_MAX_ADU 256

typedef enum {
	MB_OK = 0,
	MB_ERR_CRC,
	MB_ERR_ADDR,
	MB_ERR_LEN,
	MB_ERR_FC_UNSUPPORTED,
	MB_ERR_INTERNAL
} mb_result_t;

/**
 * Handler-Signatur:
 * - req_pdu zeigt auf [addr][fc][data...], Länge = req_len
 * - resp_pdu: hier baust du die Antwort (OHNE CRC). Gib resp_len zurück.
 * - Rückgabe:
 *   - MB_OK -> Engine hängt CRC an und sendet
 *   - MB_ERR_* -> Engine kann Exception senden (optional) oder ignorieren
 */

typedef mb_result_t (*mb_fc_handler_t)(const uint8_t *req_pdu, uint16_t req_len,
		uint8_t *resp_pdu, uint16_t *resp_len, void *user_ctx);

typedef struct {
	UART_HandleTypeDef *huart;

	// RS485 DE (optional). Wenn de_port == NULL -> kein DE handling.
	GPIO_TypeDef *de_port;
	uint16_t de_pin;

	uint8_t slave_id;

	// RX
	uint8_t rx_buf[MB_MAX_ADU];
	uint16_t rx_len;
	volatile uint8_t frame_ready;

	// Handlers pro Function Code (0..255)
	mb_fc_handler_t fc_table[256];
	void *user_ctx;
} ModbusSlave;

void mb_init(ModbusSlave *mb, UART_HandleTypeDef *huart, uint8_t slave_id,
		GPIO_TypeDef *de_port, uint16_t de_pin, void *user_ctx);

void mb_start_rx(ModbusSlave *mb);

void mb_register_fc(ModbusSlave *mb, uint8_t function_code,
		mb_fc_handler_t handler);

/**
 * Call from HAL callback:
 * HAL_UARTEx_RxEventCallback(huart, Size)
 */
void mb_on_rx_idle(ModbusSlave *mb, UART_HandleTypeDef *huart, uint16_t Size);

/**
 * Call in while(1):
 * - verarbeitet ein fertiges Frame und sendet ggf. Antwort
 */
void mb_poll(ModbusSlave *mb);

void sendDataRS485int(uint8_t *data);
void HAL_UartEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* INC_STEPPER_H */
