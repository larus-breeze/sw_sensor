#include "system_configuration.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS_wrapper.h"
#include "xbusdef.h"
#include "mtssp_interface.h"
#include "mtssp_driver_spi.h"
#include "xbusmessageid.h"

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

#define DATA_BUFSIZE_BYTES 256

/*!	\brief Read data from the Notification and Control pipes of the device
*/
void readDataFrom_MTI( MtsspInterface* device, uint8_t * dataBuffer)
{
	uint16_t notificationMessageSize;
	uint16_t measurementMessageSize;
	device->readPipeStatus(notificationMessageSize, measurementMessageSize);

	dataBuffer[0] = XBUS_PREAMBLE;
	dataBuffer[1] = XBUS_MASTERDEVICE;

	if (notificationMessageSize && notificationMessageSize < DATA_BUFSIZE_BYTES)
	{
		device->readFromPipe(&dataBuffer[2], notificationMessageSize, XBUS_NOTIFICATION_PIPE);
//		handleEvent(EVT_XbusMessage, m_dataBuffer);
	}

	if (measurementMessageSize && measurementMessageSize < DATA_BUFSIZE_BYTES)
	{
		device->readFromPipe(&dataBuffer[2], measurementMessageSize, XBUS_MEASUREMENT_PIPE);
//		handleEvent(EVT_XbusMessage, m_dataBuffer);
		heartbeat();
	}
}

static ROM uint8_t xbusData[] = {0x20, 0x30, 0x00, 0x0A};	// (Output mode: Euler angles (0x2030) at 10 Hz)

void configure_MTI_data_output( MtsspInterface* m_device)
{
	XbusMessage msg(XMID_SetOutputConfig);
	msg.m_length = sizeof(xbusData);
	msg.m_data = (uint8_t *)xbusData;
	m_device->sendXbusMessage(&msg);
}

/*!	\brief Returns the value of the DataReady line
*/
static bool checkDataReadyLine()
{
	return HAL_GPIO_ReadPin( IMU_PORT, IMU_DRDY ) == GPIO_PIN_SET;
}

static void run( void *)
{
	uint8_t dataBuffer[DATA_BUFSIZE_BYTES];
	MtsspDriverSpi SPI_driver;
	MtsspInterface IMU_interface( & SPI_driver);

	init_ports_and_reset_mti();

	configure_MTI_data_output( &IMU_interface);

	for( synchronous_timer t(10); true; t.sync())
	{
		readDataFrom_MTI( &IMU_interface, dataBuffer);
	}
}
#define STACKSIZE 512

uint32_t __ALIGNED(STACKSIZE) stack_buffer[STACKSIZE];

static TaskParameters_t p =
{
		run,
		"MTI_drv",
		STACKSIZE,
		0,
		STANDARD_TASK_PRIORITY,
		stack_buffer,
	{
		{ COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
		{ 0, 0, 0 },
		{ 0, 0, 0 }
	}
};

RestrictedTask mti_driver( p);


