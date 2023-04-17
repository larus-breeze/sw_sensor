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

void SPI_Init(SPI_HandleTypeDef *hspi);
void SPI_Transceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
void SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t Size,  uint32_t timeout=0);
void SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t Size, uint32_t timeout=0);

#ifdef __cplusplus
}
#endif


#endif /* CUSTOM_SPI_H_ */
