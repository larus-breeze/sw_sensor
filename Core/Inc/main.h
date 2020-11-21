/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define SD_DETECT_Pin GPIO_PIN_13
#define SD_DETECT_GPIO_Port GPIOC
#define SUPPLY_SENSE_Pin GPIO_PIN_0
#define SUPPLY_SENSE_GPIO_Port GPIOC
#define L3GD20_INT1_Pin GPIO_PIN_0
#define L3GD20_INT1_GPIO_Port GPIOB
#define L3GD20_INT2_Pin GPIO_PIN_1
#define L3GD20_INT2_GPIO_Port GPIOB
#define MTi_1IMU_PSEL0_Pin GPIO_PIN_10
#define MTi_1IMU_PSEL0_GPIO_Port GPIOD
#define MTi_1IMU_PSEL1_Pin GPIO_PIN_11
#define MTi_1IMU_PSEL1_GPIO_Port GPIOD
#define MTi_1IMU_DRDY_Pin GPIO_PIN_12
#define MTi_1IMU_DRDY_GPIO_Port GPIOD
#define MTi_1IMU_NRST_Pin GPIO_PIN_13
#define MTi_1IMU_NRST_GPIO_Port GPIOD
#define GPS_RESETN_Pin GPIO_PIN_15
#define GPS_RESETN_GPIO_Port GPIOD
#define FXOS8700_INT1_Pin GPIO_PIN_8
#define FXOS8700_INT1_GPIO_Port GPIOA
#define FXOS8700_INT2_Pin GPIO_PIN_15
#define FXOS8700_INT2_GPIO_Port GPIOA
#define LED_STATUS1_Pin GPIO_PIN_3
#define LED_STATUS1_GPIO_Port GPIOD
#define LED_STATUS2_Pin GPIO_PIN_4
#define LED_STATUS2_GPIO_Port GPIOD
#define LED_STATUS3_Pin GPIO_PIN_5
#define LED_STATUS3_GPIO_Port GPIOD
#define BL_RESETB_Pin GPIO_PIN_7
#define BL_RESETB_GPIO_Port GPIOD
#define FXOS8700_RST_Pin GPIO_PIN_4
#define FXOS8700_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
