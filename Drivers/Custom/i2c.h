/*
 * i2c.h
 *
 *  Created on: 03.12.2020
 *      Author: mbetz
 */

#ifndef CUSTOM_I2C_H_
#define CUSTOM_I2C_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

void I2C_Init(void);
void I2C_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
void I2C_ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
void I2C_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
void I2C_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_I2C_H_ */
