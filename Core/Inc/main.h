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
#define V02_Pin GPIO_PIN_2
#define V02_GPIO_Port GPIOE
#define EN_Pin GPIO_PIN_4
#define EN_GPIO_Port GPIOE
#define SYS_CONF_0_Pin GPIO_PIN_6
#define SYS_CONF_0_GPIO_Port GPIOE
#define EAN_Pin GPIO_PIN_13
#define EAN_GPIO_Port GPIOC
#define V05_Pin GPIO_PIN_0
#define V05_GPIO_Port GPIOC
#define Vopamp1_Pin GPIO_PIN_1
#define Vopamp1_GPIO_Port GPIOC
#define VBAT_DIV_Pin GPIO_PIN_2
#define VBAT_DIV_GPIO_Port GPIOC
#define Vopamp2_Pin GPIO_PIN_3
#define Vopamp2_GPIO_Port GPIOC
#define DVS_Pin GPIO_PIN_2
#define DVS_GPIO_Port GPIOA
#define A0_MUX_Pin GPIO_PIN_1
#define A0_MUX_GPIO_Port GPIOB
#define A1_MUX_Pin GPIO_PIN_2
#define A1_MUX_GPIO_Port GPIOB
#define A2_MUX_Pin GPIO_PIN_7
#define A2_MUX_GPIO_Port GPIOE
#define A3_MUX_Pin GPIO_PIN_8
#define A3_MUX_GPIO_Port GPIOE
#define A4_MUX_Pin GPIO_PIN_9
#define A4_MUX_GPIO_Port GPIOE
#define STIND0_Pin GPIO_PIN_11
#define STIND0_GPIO_Port GPIOE
#define STIND1_Pin GPIO_PIN_13
#define STIND1_GPIO_Port GPIOE
#define SYS_CONF_4_Pin GPIO_PIN_10
#define SYS_CONF_4_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_12
#define RST_GPIO_Port GPIOB
#define WAKE_UP_Pin GPIO_PIN_15
#define WAKE_UP_GPIO_Port GPIOB
#define SW_BTN_Pin GPIO_PIN_8
#define SW_BTN_GPIO_Port GPIOD
#define V30_Pin GPIO_PIN_9
#define V30_GPIO_Port GPIOD
#define V31_Pin GPIO_PIN_10
#define V31_GPIO_Port GPIOD
#define V24_Pin GPIO_PIN_11
#define V24_GPIO_Port GPIOD
#define V23_Pin GPIO_PIN_12
#define V23_GPIO_Port GPIOD
#define V29_Pin GPIO_PIN_13
#define V29_GPIO_Port GPIOD
#define V26_Pin GPIO_PIN_14
#define V26_GPIO_Port GPIOD
#define V25_Pin GPIO_PIN_15
#define V25_GPIO_Port GPIOD
#define V27_Pin GPIO_PIN_6
#define V27_GPIO_Port GPIOC
#define V28_Pin GPIO_PIN_7
#define V28_GPIO_Port GPIOC
#define V20_Pin GPIO_PIN_8
#define V20_GPIO_Port GPIOC
#define V21_Pin GPIO_PIN_9
#define V21_GPIO_Port GPIOC
#define V22_Pin GPIO_PIN_8
#define V22_GPIO_Port GPIOA
#define V13_Pin GPIO_PIN_9
#define V13_GPIO_Port GPIOA
#define V14_Pin GPIO_PIN_10
#define V14_GPIO_Port GPIOA
#define V15_Pin GPIO_PIN_15
#define V15_GPIO_Port GPIOA
#define V16_Pin GPIO_PIN_10
#define V16_GPIO_Port GPIOC
#define V17_Pin GPIO_PIN_11
#define V17_GPIO_Port GPIOC
#define V18_Pin GPIO_PIN_12
#define V18_GPIO_Port GPIOC
#define V19_Pin GPIO_PIN_0
#define V19_GPIO_Port GPIOD
#define V08_Pin GPIO_PIN_1
#define V08_GPIO_Port GPIOD
#define V07_Pin GPIO_PIN_2
#define V07_GPIO_Port GPIOD
#define V10_Pin GPIO_PIN_3
#define V10_GPIO_Port GPIOD
#define V09_Pin GPIO_PIN_4
#define V09_GPIO_Port GPIOD
#define V12_Pin GPIO_PIN_5
#define V12_GPIO_Port GPIOD
#define V06_Pin GPIO_PIN_7
#define V06_GPIO_Port GPIOD
#define V11_Pin GPIO_PIN_3
#define V11_GPIO_Port GPIOB
#define V03_Pin GPIO_PIN_5
#define V03_GPIO_Port GPIOB
#define V04_Pin GPIO_PIN_0
#define V04_GPIO_Port GPIOE
#define V01_Pin GPIO_PIN_1
#define V01_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
