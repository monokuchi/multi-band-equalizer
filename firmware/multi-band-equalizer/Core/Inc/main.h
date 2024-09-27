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
#define CTRL_KNB_100_HZ_Pin GPIO_PIN_4
#define CTRL_KNB_100_HZ_GPIO_Port GPIOA
#define CTRL_KNB_200_HZ_Pin GPIO_PIN_5
#define CTRL_KNB_200_HZ_GPIO_Port GPIOA
#define CTRL_KNB_400_HZ_Pin GPIO_PIN_6
#define CTRL_KNB_400_HZ_GPIO_Port GPIOA
#define CTRL_KNB_800_HZ_Pin GPIO_PIN_7
#define CTRL_KNB_800_HZ_GPIO_Port GPIOA
#define CTRL_KNB_1600_HZ_Pin GPIO_PIN_4
#define CTRL_KNB_1600_HZ_GPIO_Port GPIOC
#define CTRL_KNB_3200_HZ_Pin GPIO_PIN_5
#define CTRL_KNB_3200_HZ_GPIO_Port GPIOC
#define CTRL_KNB_6400_HZ_Pin GPIO_PIN_0
#define CTRL_KNB_6400_HZ_GPIO_Port GPIOB
#define CTRL_KNB_VOL_LVL_Pin GPIO_PIN_1
#define CTRL_KNB_VOL_LVL_GPIO_Port GPIOB
#define LED_STATUS_RED_Pin GPIO_PIN_15
#define LED_STATUS_RED_GPIO_Port GPIOE
#define I2S2_LRCK_Pin GPIO_PIN_12
#define I2S2_LRCK_GPIO_Port GPIOB
#define I2S2_SCLK_Pin GPIO_PIN_13
#define I2S2_SCLK_GPIO_Port GPIOB
#define I2S2_MCLK_Pin GPIO_PIN_6
#define I2S2_MCLK_GPIO_Port GPIOC
#define CODEC_NRST_Pin GPIO_PIN_12
#define CODEC_NRST_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
