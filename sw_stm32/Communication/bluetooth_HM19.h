#ifndef BT_HM_H_
#define BT_HM_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

bool Bluetooth_Init(void);
void Bluetooth_Transmit(uint8_t *pData, uint16_t Size);
bool Bluetooth_Receive(uint8_t *pRxByte, uint32_t timeout);

#endif /* BT_HM_H_ */
