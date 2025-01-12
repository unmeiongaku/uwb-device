/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_LED_1_Pin GPIO_PIN_13
#define MCU_LED_1_GPIO_Port GPIOC
#define MCU_LED_0_Pin GPIO_PIN_14
#define MCU_LED_0_GPIO_Port GPIOC
#define MCU_BUZZER_Pin GPIO_PIN_15
#define MCU_BUZZER_GPIO_Port GPIOC
#define MCU_BTN_2_Pin GPIO_PIN_0
#define MCU_BTN_2_GPIO_Port GPIOA
#define MCU_BTN_2_EXTI_IRQn EXTI0_IRQn
#define MCU_BTN_1_Pin GPIO_PIN_1
#define MCU_BTN_1_GPIO_Port GPIOA
#define MCU_BTN_1_EXTI_IRQn EXTI1_IRQn
#define DW_NSS_Pin GPIO_PIN_4
#define DW_NSS_GPIO_Port GPIOA
#define MCU_SPI_SCK_Pin GPIO_PIN_5
#define MCU_SPI_SCK_GPIO_Port GPIOA
#define MCU_SPI_MISO_Pin GPIO_PIN_6
#define MCU_SPI_MISO_GPIO_Port GPIOA
#define MCU_SPI_MOSI_Pin GPIO_PIN_7
#define MCU_SPI_MOSI_GPIO_Port GPIOA
#define DW_PWR_Pin GPIO_PIN_15
#define DW_PWR_GPIO_Port GPIOB
#define DW_IRQn_Pin GPIO_PIN_6
#define DW_IRQn_GPIO_Port GPIOC
#define DW_EXTON_Pin GPIO_PIN_7
#define DW_EXTON_GPIO_Port GPIOC
#define DW_EXTON_EXTI_IRQn EXTI9_5_IRQn
#define DW_WAKEUP_Pin GPIO_PIN_8
#define DW_WAKEUP_GPIO_Port GPIOC
#define DW_RESET_Pin GPIO_PIN_9
#define DW_RESET_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
