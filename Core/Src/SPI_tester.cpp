/**
 * @file    SPI_tester.cpp
 * @brief   simple SPI loopback test
 * @author  Dr. Klaus Schaefer klaus.schaefer@h-da.de
 */
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "spi.h"

ROM uint8_t TX_pattern[]="1234567890_ABCDEF";
COMMON uint8_t RX_pattern[ sizeof( TX_pattern)];

static void test( void *)
{
	while( true)
	{
		SPI_Transceive(&hspi1, (uint8_t *)TX_pattern, RX_pattern, sizeof( TX_pattern));
		delay(100);
	}
}

RestrictedTask SPI_tester( test, "SPI");



