/*
 * usart_1_driver.h
 *
 *  Created on: Jan 1, 2023
 *      Author: schaefer
 */

#ifndef CUSTOM_USART_1_DRIVER_H_
#define CUSTOM_USART_1_DRIVER_H_

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdbool.h>  //For usage from C-Code

void USART_1_Init (void);
void USART_1_transmit_DMA( uint8_t *pData, uint16_t Size);
bool UART1_Receive(uint8_t *pRxByte, uint32_t timeout);
void UART1_RxCpltCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_USART_1_DRIVER_H_ */
