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
#include "stm32f1xx_hal.h"

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
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define MPPT_EN_Pin GPIO_PIN_3
#define MPPT_EN_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define LoRa_RST_Pin GPIO_PIN_0
#define LoRa_RST_GPIO_Port GPIOB
#define LoRa_EXT0_Pin GPIO_PIN_1
#define LoRa_EXT0_GPIO_Port GPIOB
#define LoRa_EXT0_EXTI_IRQn EXTI1_IRQn
#define LoRa_EXT2_Pin GPIO_PIN_10
#define LoRa_EXT2_GPIO_Port GPIOB
#define LoRa_EXT3_Pin GPIO_PIN_11
#define LoRa_EXT3_GPIO_Port GPIOB
#define LoRa_EXT4_Pin GPIO_PIN_12
#define LoRa_EXT4_GPIO_Port GPIOB
#define LoRa_EXT5_Pin GPIO_PIN_13
#define LoRa_EXT5_GPIO_Port GPIOB
#define USB_Pin GPIO_PIN_14
#define USB_GPIO_Port GPIOB
#define NRE_Pin GPIO_PIN_15
#define NRE_GPIO_Port GPIOB
#define DE_Pin GPIO_PIN_8
#define DE_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOA
#define PWR_SEN_Pin GPIO_PIN_3
#define PWR_SEN_GPIO_Port GPIOB
#define PWR_MAIN_Pin GPIO_PIN_4
#define PWR_MAIN_GPIO_Port GPIOB
#define PWR_SUB_Pin GPIO_PIN_5
#define PWR_SUB_GPIO_Port GPIOB
#define STAT1_Pin GPIO_PIN_8
#define STAT1_GPIO_Port GPIOB
#define STAT2_Pin GPIO_PIN_9
#define STAT2_GPIO_Port GPIOB


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
