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
#include "stm32h7xx_hal.h"

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
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define CTRL_KNB_1_Pin GPIO_PIN_6
#define CTRL_KNB_1_GPIO_Port GPIOA
#define CTRL_KNB_2_Pin GPIO_PIN_7
#define CTRL_KNB_2_GPIO_Port GPIOA
#define CTRL_KNB_3_Pin GPIO_PIN_4
#define CTRL_KNB_3_GPIO_Port GPIOC
#define CTRL_KNB_4_Pin GPIO_PIN_5
#define CTRL_KNB_4_GPIO_Port GPIOC
#define CTRL_KNB_5_Pin GPIO_PIN_0
#define CTRL_KNB_5_GPIO_Port GPIOB
#define CTRL_KNB_6_Pin GPIO_PIN_1
#define CTRL_KNB_6_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define CODEC_NRST_Pin GPIO_PIN_11
#define CODEC_NRST_GPIO_Port GPIOC
#define I2S1_OUT_Pin GPIO_PIN_7
#define I2S1_OUT_GPIO_Port GPIOD
#define I2S1_IN_Pin GPIO_PIN_9
#define I2S1_IN_GPIO_Port GPIOG
#define I2S1_LRCK_Pin GPIO_PIN_10
#define I2S1_LRCK_GPIO_Port GPIOG
#define I2S1_SCLK_Pin GPIO_PIN_11
#define I2S1_SCLK_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
