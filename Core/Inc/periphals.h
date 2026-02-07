#ifndef INC_STEPPER_H
#define INC_STEPPER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"   // GPIO_TypeDef, HAL_GPIO_WritePin, GPIO_PIN_*

//Stepper-Functions-----------------------------------------------------------------------------------------------------------------------
#define TIMER_FREQUENCY 10000

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

//ADC-Functions----------------------------------------------------------------------------------------------------

#define ADC_RESOLUTION 4095.0f
#define REF_Voltage (float) 3.00

typedef struct {
	float leakage_threshold;
	float value;
	uint32_t channel;
	ADC_HandleTypeDef hadc;
} conductivity;

float ADC_GetVoltage(ADC_HandleTypeDef *hadc, float vref);
GBStatus conductivity_init(conductivity *self, float leakage_threshold, ADC_HandleTypeDef hadc, uint32_t Channel);
bool conductivity_level_reached(conductivity *self);

//Flow-Sensor-Functions--------------------------------------------------------------------------------------------------

#define FLOW_TIMER_FREQUENCY 10000

typedef struct {
	uint32_t meas_time; //ms
	uint32_t pulsecounter;
	uint32_t psc;
	float ml_per_pulse;
	float min_flow; //ml/min
	float max_flow; //ml/min
	float current_flow; //ml/min
} flowmeter;

GBStatus flowmeter_init(flowmeter *self, uint32_t meas_time, float ml_per_pulse, float min_flow, float max_flow);
GBStatus flowmeter_add_pulse(flowmeter *self);
GBStatus flowmeter_calc_flow (flowmeter *self);
float flowmeter_get_flow (flowmeter *self); 	// mL/min

#endif /* INC_STEPPER_H */
