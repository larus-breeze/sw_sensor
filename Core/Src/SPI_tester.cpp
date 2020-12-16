/**
 * @file    SPI_tester.cpp
 * @brief   simple SPI loopback test
 * @author  Dr. Klaus Schaefer klaus.schaefer@h-da.de
 */
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "spi.h"

#if RUN_SPI_TESTER

#define CHIP_SELECT_PORT	GPIOA
#define CHIP_SELECT_PIN 	GPIO_PIN_4	// IMU_NSS

ROM uint8_t TX_pattern[]="1234567890_ABCDEF";
COMMON uint8_t RX_pattern[ sizeof( TX_pattern)];

static void test( void *)
{
	while( true)
	{
		HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_RESET);
		SPI_Transceive(&hspi1, (uint8_t *)TX_pattern, RX_pattern, sizeof( TX_pattern));
		HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);
		ASSERT( strcmp( (char *)RX_pattern, (char *)TX_pattern) == 0);
		delay(10);
	}
}

RestrictedTask SPI_tester( test, "SPItest");

#endif

