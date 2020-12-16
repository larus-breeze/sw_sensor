#include "system_configuration.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS_wrapper.h"
#include "xbusdef.h"
#include "mtssp_interface.h"
#include "mtssp_driver_spi.h"

#define IMU_PSEL0  GPIO_PIN_10
#define IMU_PSEL1  GPIO_PIN_11
#define IMU_DRDY   GPIO_PIN_12
#define IMU_NRST   GPIO_PIN_13
#define IMU_PORT   GPIOD

static inline void init_ports_and_reset_mti(void) // GPIO stuff
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitStruct.Pin = IMU_PSEL0 | IMU_PSEL1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(IMU_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin( IMU_PORT, IMU_PSEL0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin( IMU_PORT, IMU_PSEL1, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = IMU_NRST;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(IMU_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin( IMU_PORT, IMU_NRST, GPIO_PIN_RESET);
	delay(1);
	HAL_GPIO_WritePin( IMU_PORT, IMU_NRST, GPIO_PIN_SET);
	delay(1);
}

static COMMON uint8_t m_dataBuffer[256];
static COMMON uint8_t counter=100;

/*!	\brief Read data from the Notification and Control pipes of the device
*/
void readDataFrom_MTI( MtsspInterface* m_device)
{
	uint16_t notificationMessageSize;
	uint16_t measurementMessageSize;
	m_device->readPipeStatus(notificationMessageSize, measurementMessageSize);

	m_dataBuffer[0] = XBUS_PREAMBLE;
	m_dataBuffer[1] = XBUS_MASTERDEVICE;

	if (notificationMessageSize && notificationMessageSize < sizeof(m_dataBuffer))
	{
		m_device->readFromPipe(&m_dataBuffer[2], notificationMessageSize, XBUS_NOTIFICATION_PIPE);
//		handleEvent(EVT_XbusMessage, m_dataBuffer);
	}

	if (measurementMessageSize && measurementMessageSize < sizeof(m_dataBuffer))
	{
		m_device->readFromPipe(&m_dataBuffer[2], measurementMessageSize, XBUS_MEASUREMENT_PIPE);
//		handleEvent(EVT_XbusMessage, m_dataBuffer);
		--counter;
		if( counter ==0)
		{
			counter = 100;
			heartbeat();
		}
	}
}

/*!	\brief Returns the value of the DataReady line
*/
static bool checkDataReadyLine()
{
	return HAL_GPIO_ReadPin( IMU_PORT, IMU_DRDY ) == GPIO_PIN_SET;
}

static void run( void *)
{
	MtsspDriverSpi SPI_driver;
	MtsspInterface IMU_interface( & SPI_driver);

	init_ports_and_reset_mti();

	for( synchronous_timer t(10); true; t.sync())
	{
		readDataFrom_MTI( &IMU_interface);
	}
}

RestrictedTask mti_driver( run, "MTI_1");


