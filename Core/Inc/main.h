/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32u5xx_hal.h"

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
#define A4_MUX_Pin GPIO_PIN_5
#define A4_MUX_GPIO_Port GPIOE
#define A2_MUX_Pin GPIO_PIN_3
#define A2_MUX_GPIO_Port GPIOE
#define A0_MUX_Pin GPIO_PIN_1
#define A0_MUX_GPIO_Port GPIOE
#define LED_1_Pin GPIO_PIN_6
#define LED_1_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_5
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_2
#define LED_3_GPIO_Port GPIOD
#define RST_Pin GPIO_PIN_11
#define RST_GPIO_Port GPIOC
#define A3_MUX_Pin GPIO_PIN_4
#define A3_MUX_GPIO_Port GPIOE
#define A1_MUX_Pin GPIO_PIN_2
#define A1_MUX_GPIO_Port GPIOE
#define MA2_Pin GPIO_PIN_4
#define MA2_GPIO_Port GPIOD
#define MA0_Pin GPIO_PIN_1
#define MA0_GPIO_Port GPIOD
#define WAKE_UP_Pin GPIO_PIN_10
#define WAKE_UP_GPIO_Port GPIOC
#define MA1_Pin GPIO_PIN_3
#define MA1_GPIO_Port GPIOD
#define NA0_Pin GPIO_PIN_7
#define NA0_GPIO_Port GPIOD
#define SW_BTN_Pin GPIO_PIN_9
#define SW_BTN_GPIO_Port GPIOC
#define IMON_Pin GPIO_PIN_14
#define IMON_GPIO_Port GPIOF
#define DVS_Pin GPIO_PIN_8
#define DVS_GPIO_Port GPIOE
#define SYS_FLT_Pin GPIO_PIN_10
#define SYS_FLT_GPIO_Port GPIOE
#define MODE_Pin GPIO_PIN_12
#define MODE_GPIO_Port GPIOE
#define EN_MAX77874_Pin GPIO_PIN_7
#define EN_MAX77874_GPIO_Port GPIOE
#define SPI1_SS_Pin GPIO_PIN_4
#define SPI1_SS_GPIO_Port GPIOA
#define ROUT_N_Pin GPIO_PIN_11
#define ROUT_N_GPIO_Port GPIOE
#define RIN_N_Pin GPIO_PIN_15
#define RIN_N_GPIO_Port GPIOE
#define PWR_FLT_Pin GPIO_PIN_9
#define PWR_FLT_GPIO_Port GPIOE
#define WD_Pin GPIO_PIN_13
#define WD_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
