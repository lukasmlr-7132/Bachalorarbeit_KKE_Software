/*
 * functions.c
 *
 *  Created on: 28.01.2026
 *      Author: lukas
 */
#include "functions.h"
#include "main.h"                 // <-- GPIO_Pins & Ports
#include "stm32f4xx_hal.h"        // <-- HAL_GPIO_WritePin

void stepper_actor_init(stepper_actor *self,
                        uint16_t freq_high,
                        uint16_t freq_low,
                        uint16_t startup_ms)

{
    if (self == NULL) return;

    self->drive_frequency_high = freq_high;
    self->drive_frequency_low  = freq_low;
    self->startup_time_ms      = startup_ms;

    self->step   = 0;
    self->steppcounter   = 0;
    for (size_t i = 0; i < 6; ++i) {
        self->stepp_positions[i] = 0;
    }
    self->targetposition  = 0;

    self->recent_current_value = 0;
    self->last_current_value   = 0;
    self->current_dif          = 0;
}

void stepper_actor_set_position(stepper_actor *self, size_t index, uint32_t step_value)
{
    if (self == NULL) return;
    if (index >= 5) return;   // im Embedded oft ohne throw, nur Guard

    self->stepp_positions[index] = step_value;
}

void stepper_actor_set_target(stepper_actor *self, size_t index)
{
    if (self == NULL) return;
    if (index >= 5) return;   // im Embedded oft ohne throw, nur Guard

    self->targetposition = self->stepp_positions[index];
}

void stepper_actor_set_step(stepper_actor *self, size_t index){
	switch (index) {
			case 0:
				HAL_GPIO_WritePin(Stepper_A1_uC_GPIO_Port, Stepper_A1_uC_Pin, RESET);
				HAL_GPIO_WritePin(Stepper_A3_uC_GPIO_Port, Stepper_A3_uC_Pin,
						RESET);
				HAL_GPIO_WritePin(Stepper_B2_uC_GPIO_Port, Stepper_B2_uC_Pin,
						RESET);
				HAL_GPIO_WritePin(Stepper_B4_uC_GPIO_Port, Stepper_B4_uC_Pin, RESET);
				break;
			case 1:
				HAL_GPIO_WritePin(Stepper_A1_uC_GPIO_Port, Stepper_A1_uC_Pin, SET);
				HAL_GPIO_WritePin(Stepper_A3_uC_GPIO_Port, Stepper_A3_uC_Pin,
						RESET);
				HAL_GPIO_WritePin(Stepper_B2_uC_GPIO_Port, Stepper_B2_uC_Pin,
						RESET);
				HAL_GPIO_WritePin(Stepper_B4_uC_GPIO_Port, Stepper_B4_uC_Pin, SET);
				break;

			case 2:
				HAL_GPIO_WritePin(Stepper_A1_uC_GPIO_Port, Stepper_A1_uC_Pin, SET);
				HAL_GPIO_WritePin(Stepper_A3_uC_GPIO_Port, Stepper_A3_uC_Pin,
						RESET);
				HAL_GPIO_WritePin(Stepper_B2_uC_GPIO_Port, Stepper_B2_uC_Pin, SET);
				HAL_GPIO_WritePin(Stepper_B4_uC_GPIO_Port, Stepper_B4_uC_Pin,
						RESET);
				break;

			case 3:
				HAL_GPIO_WritePin(Stepper_A1_uC_GPIO_Port, Stepper_A1_uC_Pin,
						RESET);
				HAL_GPIO_WritePin(Stepper_A3_uC_GPIO_Port, Stepper_A3_uC_Pin, SET);
				HAL_GPIO_WritePin(Stepper_B2_uC_GPIO_Port, Stepper_B2_uC_Pin, SET);
				HAL_GPIO_WritePin(Stepper_B4_uC_GPIO_Port, Stepper_B4_uC_Pin,
						RESET);
				break;

			case 4:
				HAL_GPIO_WritePin(Stepper_A1_uC_GPIO_Port, Stepper_A1_uC_Pin,
						RESET);
				HAL_GPIO_WritePin(Stepper_A3_uC_GPIO_Port, Stepper_A3_uC_Pin, SET);
				HAL_GPIO_WritePin(Stepper_B2_uC_GPIO_Port, Stepper_B2_uC_Pin,
						RESET);
				HAL_GPIO_WritePin(Stepper_B4_uC_GPIO_Port, Stepper_B4_uC_Pin, SET);
				break;

	default:
		break;
}
}

uint32_t stepper_actor_get_position(const stepper_actor *self, size_t index)
{
    if (self == NULL) return 0;
    if (index >= 5) return 0;

    return self->stepp_positions[index];
}

void stepper_actor_get_increase_step(stepper_actor *self)
{
	if (self->step == 0) {
		self->step = 1;
	} else {
		self->step += 1;
		if (self->step > 4) self->step = 1;
	}

	self->steppcounter += 1;
	stepper_actor_set_step(self, self->step);
}

void stepper_actor_get_decrease_step(stepper_actor *self)
{
	if (self->step == 0) {
		return;
	}

	if (self->step == 1) {
		self->step = 4;
	} else {
		self->step -= 1;
	}

	if (self->steppcounter > 0) {
		self->steppcounter -= 1;
	}

	stepper_actor_set_step(self, self->step);
}

void stepper_actor_update_current(stepper_actor *self, uint32_t current_value)
{
    if (self == NULL) return;

    self->last_current_value = self->recent_current_value;
    self->recent_current_value = current_value;

    if (self->recent_current_value >= self->last_current_value) {
        self->current_dif = self->recent_current_value - self->last_current_value;
    } else {
        self->current_dif = self->last_current_value - self->recent_current_value;
    }
}




