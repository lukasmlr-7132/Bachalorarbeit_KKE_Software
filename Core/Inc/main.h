/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Level_Sense_PWM_Pin GPIO_PIN_5
#define Level_Sense_PWM_GPIO_Port GPIOE
#define Stepper_B4_uC_Pin GPIO_PIN_13
#define Stepper_B4_uC_GPIO_Port GPIOC
#define Stepper_A1_uC_Pin GPIO_PIN_14
#define Stepper_A1_uC_GPIO_Port GPIOC
#define Stepper_A3_uC_Pin GPIO_PIN_15
#define Stepper_A3_uC_GPIO_Port GPIOC
#define Level_Sense_Signal_Pin GPIO_PIN_0
#define Level_Sense_Signal_GPIO_Port GPIOC
#define Stepper_Current_A_Pin GPIO_PIN_1
#define Stepper_Current_A_GPIO_Port GPIOA
#define Leackage_Sense_Signal_Pin GPIO_PIN_2
#define Leackage_Sense_Signal_GPIO_Port GPIOA
#define TWZ_Input_1_Pin GPIO_PIN_9
#define TWZ_Input_1_GPIO_Port GPIOE
#define TWZ_Input_1_EXTI_IRQn EXTI9_5_IRQn
#define debug_fault_Pin GPIO_PIN_11
#define debug_fault_GPIO_Port GPIOE
#define debug_heartbeat_Pin GPIO_PIN_12
#define debug_heartbeat_GPIO_Port GPIOE
#define debug_1_Pin GPIO_PIN_13
#define debug_1_GPIO_Port GPIOE
#define debug_2_Pin GPIO_PIN_14
#define debug_2_GPIO_Port GPIOE
#define debug_3_Pin GPIO_PIN_15
#define debug_3_GPIO_Port GPIOE
#define Leckage_Measuring_PWM_Pin GPIO_PIN_12
#define Leckage_Measuring_PWM_GPIO_Port GPIOD
#define TWZ_Input_2_Pin GPIO_PIN_7
#define TWZ_Input_2_GPIO_Port GPIOC
#define TWZ_Input_2_EXTI_IRQn EXTI9_5_IRQn
#define Stepper_B2_uC_Pin GPIO_PIN_15
#define Stepper_B2_uC_GPIO_Port GPIOA
#define UART_4_TX_Pin GPIO_PIN_10
#define UART_4_TX_GPIO_Port GPIOC
#define UART_4_RX_Pin GPIO_PIN_11
#define UART_4_RX_GPIO_Port GPIOC
#define RS485_Direction_Pin GPIO_PIN_4
#define RS485_Direction_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
