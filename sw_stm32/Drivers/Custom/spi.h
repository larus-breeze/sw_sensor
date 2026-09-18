/*
 * spi.h
 *
 *  Created on: 24.11.2020
 *      Author: mbetz
 */

#ifndef CUSTOM_SPI_H_
#define CUSTOM_SPI_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

// Notification values for the SPI2 user task, sent from spi.cpp's ISR
// callbacks - see wlan_link_handler.cpp's arm_*() functions.
#define WLAN_LINK_SPI2_NOTIFY_HALF_COMPLETE  0u
#define WLAN_LINK_SPI2_NOTIFY_FULL_COMPLETE  1u
#define WLAN_LINK_SPI2_NOTIFY_ERROR          2u

void SPI_Init(SPI_HandleTypeDef *hspi);
void SPI_Transceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
void SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t Size,  uint32_t timeout=0);
void SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t Size, uint32_t timeout=0);
void register_SPI_usertask(SPI_HandleTypeDef *hspi);

//!< Notifies whichever task last called register_SPI_usertask(&hspi2), from
//!< ISR context - lets wlan_link_handler.cpp trigger the same recovery
//!< path from a hardware signal, not just a HAL-detected error/timeout.
void notify_SPI2_task_from_ISR(uint32_t notification_value);

// Like SPI_Receive(), but returns false on timeout instead of asserting -
// for slave-mode links where "nothing sent yet" is normal.
bool SPI_Receive_Timeout(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t Size, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif


#endif /* CUSTOM_SPI_H_ */
