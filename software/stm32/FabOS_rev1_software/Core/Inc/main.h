/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define USR_LED_1_Pin GPIO_PIN_2
#define USR_LED_1_GPIO_Port GPIOE
#define USR_LED_2_Pin GPIO_PIN_3
#define USR_LED_2_GPIO_Port GPIOE
#define USR_LED_3_Pin GPIO_PIN_4
#define USR_LED_3_GPIO_Port GPIOE
#define USR_BTN_3_Pin GPIO_PIN_5
#define USR_BTN_3_GPIO_Port GPIOE
#define USR_BTN_2_Pin GPIO_PIN_6
#define USR_BTN_2_GPIO_Port GPIOE
#define USR_BTN_1_Pin GPIO_PIN_13
#define USR_BTN_1_GPIO_Port GPIOC
#define NRST_ETH_Pin GPIO_PIN_6
#define NRST_ETH_GPIO_Port GPIOA
#define EMERGENCY_STOP_Pin GPIO_PIN_15
#define EMERGENCY_STOP_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_10
#define VCP_RX_GPIO_Port GPIOC
#define VCP_RXC11_Pin GPIO_PIN_11
#define VCP_RXC11_GPIO_Port GPIOC
#define LIN_EN_Pin GPIO_PIN_4
#define LIN_EN_GPIO_Port GPIOB
#define UART_LIN_TX_Pin GPIO_PIN_6
#define UART_LIN_TX_GPIO_Port GPIOB
#define UART_LIN_RX_Pin GPIO_PIN_7
#define UART_LIN_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
