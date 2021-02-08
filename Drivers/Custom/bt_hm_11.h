/*
 * i2c.h
 *
 *  Created on: 08.02.2021
 *      Author: Maximilian Betz
 */

#ifndef BT_HM_11_H_
#define BT_HM_11_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>

bool Bluetooth_Init(void);
void Bluetooth_Transmit(uint8_t *pData, uint16_t Size);
bool Bluetooth_Receive(uint8_t *pRxByte);

#ifdef __cplusplus
}
#endif


#endif /* BT_HM_11_H_ */
