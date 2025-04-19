/**
 * @file        main.h
 * @brief       Definitions for our hardware
 * @author	Dr. Klaus Schaefer
 * @copyright 	Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "system_configuration.h"
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
void heartbeat(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_DETECT_Pin GPIO_PIN_13
#define SD_DETECT_GPIO_Port GPIOC
#define SUPPLY_SENSE_Pin GPIO_PIN_0
#define SUPPLY_SENSE_GPIO_Port GPIOC
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define SPI2_NSS_Pin GPIO_PIN_4
#define SPI2_NSS_GPIO_Port GPIOB
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
#define LED_STATUS1_Pin GPIO_PIN_3
#define LED_STATUS1_GPIO_Port GPIOD
#define LED_STATUS2_Pin GPIO_PIN_4
#define LED_STATUS2_GPIO_Port GPIOD
#define LED_STATUS3_Pin GPIO_PIN_5
#define LED_STATUS3_GPIO_Port GPIOD
#define LED_ERROR_Pin GPIO_PIN_6
#define LED_ERROR_GPIO_Port GPIOD
#define BL_RESETB_Pin GPIO_PIN_7
#define BL_RESETB_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
