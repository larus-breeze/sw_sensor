
//  Copyright (c) 2003-2020 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "mtssp_driver_spi.h"
#include "stm32f4xx_hal.h"
#include "spi.h"

#if 0
#include "board.h"
#include "init_spi.h"
#include "wait.h"
#endif
/*!	\class MtsspDriverSpi
	\brief MtsspDriver for the SPI bus
*/

#define TIMEOUT_MS 100 // dummy / unused

#define CHIP_SELECT_PORT	GPIOA
#define CHIP_SELECT_PIN 	GPIO_PIN_4	// IMU_NSS

extern SPI_HandleTypeDef hspi1;

void wait_us(unsigned)
{
	taskYIELD(); // workaround for short delay
}

static COMMON 	uint8_t buffer[4];

/*!	\brief Perform a blocking write transfer on the SPI bus
	\param[in] opcode Opcode to use
	\param[in] data Pointer to data to be written
	\param[in] dataLength Number of data bytes to write
*/
void MtsspDriverSpi::write(uint8_t opcode, uint8_t const* data, int dataLength)
{
	HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_RESET);
	wait_us(4);
	buffer[0] = opcode;
	buffer[1] = 0;
	buffer[2] = 0;
	buffer[3] = 0;
	SPI_Transmit(&hspi1, buffer, sizeof(buffer), TIMEOUT_MS);
	SPI_Transmit(&hspi1, (uint8_t*)data, dataLength, TIMEOUT_MS);
	wait_us(4);
	HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);
}

/*!	\brief Perform a blocking read transfer on the SPI bus
	\param[in] opcode Opcode to use
	\param[out] data Pointer to result buffer
	\param[in] dataLength Number of data bytes to read
*/
void MtsspDriverSpi::read(uint8_t opcode, uint8_t* dest, int dataLength)
{
	HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_RESET);
	wait_us(4);
	buffer[0] = opcode;
	buffer[1] = 0;
	buffer[2] = 0;
	buffer[3] = 0;
	SPI_Transmit(&hspi1, buffer, sizeof(buffer), TIMEOUT_MS);
	SPI_Receive(&hspi1, dest, dataLength, TIMEOUT_MS);
	wait_us(4);
	HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);
}


/*!	\brief Perform a blocking write transfer on the SPI bus
	\param[in] data Pointer to data to be written
	\param[in] dataLength Number of data bytes to write
*/
void MtsspDriverSpi::writeRaw(uint8_t const* data, int dataLength)
{
	HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_RESET);
	wait_us(4);
	SPI_Transmit(&hspi1, (uint8_t*)data, dataLength, TIMEOUT_MS);
	wait_us(4);
	HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);
}





