/*
 * uart6.h
 *
 *  Created on: 03.01.2020
 *      Author: mbetz
 */

#ifndef CUSTOM_UART6_H_
#define CUSTOM_UART6_H_

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdbool.h>  //For the usage from C-Code
#include "stm32f4xx_hal.h"

void UART6_Init(void);
void UART6_DeInit(void);
void UART6_ChangeBaudRate(uint32_t rate);
void UART6_Transmit(const uint8_t *pData, uint16_t Size);
bool UART6_Receive(uint8_t *pRxByte, uint32_t timeout);
void UART6_RxCpltCallback(void);
extern UART_HandleTypeDef huart6;

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_UART6_H_ */
