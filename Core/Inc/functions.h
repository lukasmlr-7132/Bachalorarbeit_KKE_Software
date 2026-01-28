/*
 * fuctions.h
 *
 *  Created on: 28.01.2026
 *      Author: lukas
 */

#ifndef INC_FUNCTIONS_H
#define INC_FUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#pragma once
#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"


typedef struct {
    uint16_t drive_frequency_high; // Hz
    uint16_t drive_frequency_low;  // Hz
    uint16_t startup_time_ms;      // startup ramp time (ms)

    uint8_t step;
    uint32_t steppcounter;         // number of steps
    uint32_t stepp_positions[6];   // positions of 0..5 for process
    uint32_t targetposition;
    uint8_t stepp_pattern [4];

    uint32_t recent_current_value;
    uint32_t last_current_value;
    uint32_t current_dif;
} stepper_actor;

/* "Methoden" */
void stepper_actor_init(stepper_actor *self,
                        uint16_t freq_high,
						uint16_t freq_low,
						uint16_t startup_ms);


void stepper_actor_set_position(stepper_actor *self, size_t index, uint32_t step_value);
uint32_t stepper_actor_get_position(const stepper_actor *self, size_t index);

void stepper_actor_set_target(stepper_actor *self, size_t index);
void stepper_actor_set_step(stepper_actor *self, size_t index);

void stepper_actor_get_increase_step(stepper_actor *self);
void stepper_actor_get_decrease_step(stepper_actor *self);

void stepper_actor_update_current(stepper_actor *self, uint32_t current_value);
uint32_t stepper_actor_get_current_diff(const stepper_actor *self);


#ifdef __cplusplus
}
#endif
#endif /* INC_FUNCTIONS_H */

