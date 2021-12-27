/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW3_Pin GPIO_PIN_13
#define SW3_GPIO_Port GPIOC
#define SW3_EXTI_IRQn EXTI4_15_IRQn
#define SW4_Pin GPIO_PIN_14
#define SW4_GPIO_Port GPIOC
#define SW4_EXTI_IRQn EXTI4_15_IRQn
#define LED_N_3_10_14_15_Pin GPIO_PIN_15
#define LED_N_3_10_14_15_GPIO_Port GPIOC
#define LED_N_2_6_9_Pin GPIO_PIN_0
#define LED_N_2_6_9_GPIO_Port GPIOF
#define LED_N_1w_5_8_12_Pin GPIO_PIN_1
#define LED_N_1w_5_8_12_GPIO_Port GPIOF
#define LED_P_7_8_9_10_Pin GPIO_PIN_0
#define LED_P_7_8_9_10_GPIO_Port GPIOA
#define LED_P_4_5_6_15_Pin GPIO_PIN_1
#define LED_P_4_5_6_15_GPIO_Port GPIOA
#define LED_N_1r_4_7_11_Pin GPIO_PIN_2
#define LED_N_1r_4_7_11_GPIO_Port GPIOA
#define LED_P_1r_1w_2_3_Pin GPIO_PIN_3
#define LED_P_1r_1w_2_3_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_4
#define SW2_GPIO_Port GPIOA
#define SW2_EXTI_IRQn EXTI4_15_IRQn
#define SW1_Pin GPIO_PIN_5
#define SW1_GPIO_Port GPIOA
#define SW1_EXTI_IRQn EXTI4_15_IRQn
#define LED_P_11_12_14_Pin GPIO_PIN_7
#define LED_P_11_12_14_GPIO_Port GPIOA
#define T_SENSOR_Pin GPIO_PIN_1
#define T_SENSOR_GPIO_Port GPIOB
#define BUZZ_PIN_Pin GPIO_PIN_5
#define BUZZ_PIN_GPIO_Port GPIOB
#define PWR220_Pin GPIO_PIN_8
#define PWR220_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
