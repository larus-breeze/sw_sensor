/*
 * i2c.h
 *
 *  Created on: 03.12.2020
 *      Author: mbetz
 */

#ifndef CUSTOM_I2C_H_
#define CUSTOM_I2C_H_

#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

void I2C_Init();
void I2C_Read();
void I2C_Write();


#endif /* CUSTOM_I2C_H_ */
