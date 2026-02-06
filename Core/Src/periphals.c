/*
 * functions.c
 *
 *  Created on: 28.01.2026
 *      Author: lukas
 */

#include "periphals.h"   // dein Header
#include <string.h>    // memcpy
#include "stm32f4xx_hal.h"
#include "main.h"

//Stepper_Motor----------------------------------------------------------------------------------------------------------------------
uint8_t beep = 1;

static const uint8_t Fullstep_Dualphase[4] = { 0b1001, // A1 + B2
		0b1010, // B2 + A3
		0b0110, // A3 + B4
		0b0101  // B4 + A1
		};

static const uint32_t full_rotation = 18000 - 1;

static void apply_pattern(const stepper *s, uint8_t p) {
	HAL_GPIO_WritePin(s->pins.A_port, s->pins.A_pin,
			(p & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(s->pins.B_port, s->pins.B_pin,
			(p & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(s->pins.C_port, s->pins.C_pin,
			(p & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(s->pins.D_port, s->pins.D_pin,
			(p & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void coils_off(const stepper *s) {
	HAL_GPIO_WritePin(s->pins.A_port, s->pins.A_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(s->pins.B_port, s->pins.B_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(s->pins.C_port, s->pins.C_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(s->pins.D_port, s->pins.D_pin, GPIO_PIN_RESET);
}

GBStatus stepper_init_hardware(stepper *self, const StepperPins *pins) {
	if (!self || !pins) {
		return GB_ERR_NULL;
	}

	// --- Hardware binding ---
	self->pins = *pins;

	// --- Vollschritt-Pattern setzen
	for (uint8_t i = 0; i < 4; i++)
		self->step_pattern[i] = Fullstep_Dualphase[i];

	// --- Sinnvolle Defaults für ALLE Felder ---
	self->drive_frequency_high_hz = 0;
	self->drive_frequency_low_hz = 0;
	self->start_sequence = 1;

	self->mode = STEPPER_OFF;
	self->step_idx = 0;
	self->step_counter = 0;

	self->target_position = 0;
	for (size_t i = 0; i < 6; i++)
		self->target_positions[i] = 0;

	self->current_recent = 0;
	self->current_last = 0;
	self->current_diff = 0;
	self->current_sum = 0;
	self->stall_limit = 0;

	coils_off(self);

	return GB_OK;
}

GBStatus stepper_init_parameters(stepper *self, uint16_t freq_high_hz,
		uint16_t freq_low_hz, uint16_t base_frequency, uint32_t stall_limit) {
	if (!self) {
		return GB_ERR_NULL;
	}

	// Minimalvalidierung (ohne Rampe/Timing-Logik)
	if (freq_high_hz == 0 || freq_low_hz == 0) {
		return GB_ERR_RANGE;
	}

	self->drive_frequency_high_hz = freq_high_hz;
	self->drive_frequency_low_hz = freq_low_hz;
	self->base_frequency = base_frequency;

	self->drive_psc = base_frequency / freq_low_hz;

	self->stall_limit = stall_limit;

	return GB_OK;
}

void stepper_sound(stepper *self) {
	// Beep-Modus aktivieren (um Rampe zu umgehen)
	beep = 1;

	self->drive_frequency_high_hz = 5000;   // 4..10 kHz testen
	stepper_set_drivefreq_high(self);

	// kurzes Ziel, damit es nur "piept" und nicht wegfährt
	self->target_position = 4000;
	self->mode = STEPPER_TARGET_OPERATION;
	self->start_sequence = 0;               // nur zur Sicherheit

	HAL_Delay(600);

	self->drive_frequency_high_hz = 10000;   // 4..10 kHz testen
	stepper_set_drivefreq_high(self);

	// kurzes Ziel, damit es nur "piept" und nicht wegfährt
	self->target_position = 0;
	self->mode = STEPPER_TARGET_OPERATION;
	self->start_sequence = 0;               // nur zur Sicherheit

	HAL_Delay(600);
	/*
	self->drive_frequency_high_hz = 5000;   // 4..10 kHz testen
	stepper_set_drivefreq_high(self);

	// kurzes Ziel, damit es nur "piept" und nicht wegfährt
	self->target_position = 4000;
	self->mode = STEPPER_TARGET_OPERATION;
	self->start_sequence = 0;               // nur zur Sicherheit

	HAL_Delay(1000);

	/*
	 self->drive_frequency_high_hz = 1500;   // 4..10 kHz testen
	 stepper_set_drivefreq_high(self);

	 // kurzes Ziel, damit es nur "piept" und nicht wegfährt
	 self->target_position = 4000;
	 self->mode = STEPPER_TARGET_OPERATION;
	 self->start_sequence = 0;               // nur zur Sicherheit

	 HAL_Delay(800);

	 self->drive_frequency_high_hz = 1000;   // 4..10 kHz testen
	 stepper_set_drivefreq_high(self);

	 // kurzes Ziel, damit es nur "piept" und nicht wegfährt
	 self->target_position = 0;
	 self->mode = STEPPER_TARGET_OPERATION;
	 self->start_sequence = 0;               // nur zur Sicherheit

	 HAL_Delay(800);

	 self->drive_frequency_high_hz = 500;   // 4..10 kHz testen
	 stepper_set_drivefreq_high(self);

	 // kurzes Ziel, damit es nur "piept" und nicht wegfährt
	 self->target_position = 4000;
	 self->mode = STEPPER_TARGET_OPERATION;
	 self->start_sequence = 0;               // nur zur Sicherheit

	 HAL_Delay(800);

	 stepper_disable(self);
	 */
	// zurück in Normalbetrieb
	beep = 0;
}

GBStatus stepper_init_positions(stepper *self,
		const uint32_t target_positions[6]) {
	if (!self || !target_positions) {
		return GB_ERR_NULL;
	}

	for (size_t i = 0; i < 6; i++) {
		self->target_positions[i] = target_positions[i];
	}

	return GB_OK;
}

GBStatus stepper_set_target(stepper *self, size_t index) {
	if (!self) {
		return GB_ERR_NULL;
	}
	if (index >= 6) {
		return GB_ERR_RANGE;
	}

	self->target_position = self->target_positions[index];
	self->mode = STEPPER_TARGET_OPERATION;
	self->start_sequence = 1;

	return GB_OK;
}

GBStatus stepper_set_drivefreq_low(stepper *self) {
	if (!self) {
		return GB_ERR_NULL;
	}

	self->drive_psc = self->base_frequency / self->drive_frequency_low_hz;

	return GB_OK;
}

GBStatus stepper_set_drivefreq_high(stepper *self) {
	if (!self) {
		return GB_ERR_NULL;
	}

	self->drive_psc = self->base_frequency / self->drive_frequency_high_hz;

	return GB_OK;
}

GBStatus stepper_disable(stepper *self) {
	if (!self) {
		return GB_ERR_NULL;
	}

	self->mode = STEPPER_OFF;
	coils_off(self);

	return GB_OK;
}

GBStatus stepper_reference(stepper *self) {
	if (!self) {
		return GB_ERR_NULL;
	}

	self->mode = STEPPER_REFERENCING;
	self->step_idx = 0;
	self->step_counter = 0;
	self->target_position = 0;
	self->start_sequence = 1;

	return GB_OK;
}

GBStatus stepper_step_ccw(stepper *self) {
	if (!self) {
		return GB_ERR_NULL;
	}

	// +1 Schritt (vorwärts)
	if (self->step_idx > 0) {
		self->step_idx--;
	} else {
		self->step_idx = 3;
	}

	if (self->step_counter > 0) {
		self->step_counter--;
	} else {
		self->step_counter = full_rotation;
	}

	apply_pattern(self, self->step_pattern[self->step_idx]);
	return GB_OK;
}

GBStatus stepper_step_cw(stepper *self) {
	if (!self) {
		return GB_ERR_NULL;
	}

	if (self->step_idx < 3) {
		self->step_idx++;
	} else {
		self->step_idx = 0;
	}

	if (self->step_counter >= full_rotation) {
		self->step_counter = 0;
	} else {
		self->step_counter++;
	}

	apply_pattern(self, self->step_pattern[self->step_idx]);
	return GB_OK;
}

void stepper_tick(stepper *self) {
	if (!self) {
		return;
	}

	if (self->mode == STEPPER_TARGET_OPERATION) {
		if (self->step_counter == self->target_position) {
			self->mode = STEPPER_OFF;
			coils_off(self);
			return;
		}

		if (self->step_counter != self->target_position) {
			(void) stepper_step_cw(self);
		}
	}

	if (self->mode == STEPPER_REFERENCING) {
		(void) stepper_step_ccw(self);
	}

	if (self->mode == STEPPER_OFF) {
		return;
	}

}

//Leackage- and Hight-Sensing--------------------------------------------------------------------------------------------------------------
float ADC_GetVoltage(ADC_HandleTypeDef *hadc, float vref) {
	uint32_t raw = HAL_ADC_GetValue(hadc);

	return ((float) raw / (float) ADC_RESOLUTION) * vref;
}

GBStatus conductivity_init(conductivity *self, float leakage_threshold, ADC_HandleTypeDef hadc, uint32_t Channel) {
	if (!self) {
		return GB_ERR_NULL;
	}

	self->leakage_threshold = leakage_threshold;
	self->hadc = hadc;
	self->channel=Channel;
	self->value = 0.0f;

	return GB_OK;
}

bool conductivity_level_reached(conductivity *self) {
	if (!self) {
		return false;
	}

	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = self->channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;

	if (HAL_ADC_ConfigChannel(&self->hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_ADC_Start(&self->hadc);

	if (HAL_ADC_PollForConversion(&self->hadc, 10) != HAL_OK) {
		HAL_ADC_Stop(&self->hadc);
		return false;
	}

	self->value = ADC_GetVoltage(&self->hadc, REF_Voltage);

	HAL_ADC_Stop(&self->hadc);

	if (self->value > self->leakage_threshold) {
		return true;
	} else {
		return false;
	}
}

//Flow-Sensor------------------------------------------------------------------------------------------------------------------------
