/*
 * usart_1_driver.h
 *
 *  Created on: Jan 1, 2023
 *      Author: schaefer
 */

#ifndef CUSTOM_USART_1_DRIVER_H_
#define CUSTOM_USART_1_DRIVER_H_

void USART_1_Init (void);
void USART_1_transmit_DMA( uint8_t *pData, uint16_t Size);

#endif /* CUSTOM_USART_1_DRIVER_H_ */
