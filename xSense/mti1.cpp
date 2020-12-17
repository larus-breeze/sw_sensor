#include "system_configuration.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS_wrapper.h"
#include "xbusdef.h"
#include "mtssp_interface.h"
#include "mtssp_driver_spi.h"
#include "xbusmessageid.h"
#include "cmsis_gcc.h"
#include "stdint.h"

#define IMU_PSEL0  GPIO_PIN_10
#define IMU_PSEL1  GPIO_PIN_11
#define IMU_DRDY   GPIO_PIN_12
#define IMU_NRST   GPIO_PIN_13
#define IMU_PORT   GPIOD

COMMON float acc[3];
COMMON float mag[3];
COMMON float gyro[3];

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

typedef union{ float f; uint32_t u;} float_word;

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
	}

	if (measurementMessageSize && measurementMessageSize < DATA_BUFSIZE_BYTES)
	{
		device->readFromPipe(&dataBuffer[2], measurementMessageSize, XBUS_MEASUREMENT_PIPE);
		if(dataBuffer[4]==0x40 && dataBuffer[0x13]==0x80 && dataBuffer[0x22]==0xC0)
		{
			float_word x;
			x.u=__REV( *(uint32_t*)(dataBuffer+0x07+0)), acc[0]=x.f;
			x.u=__REV( *(uint32_t*)(dataBuffer+0x07+4)), acc[1]=x.f;
			x.u=__REV( *(uint32_t*)(dataBuffer+0x07+8)), acc[2]=x.f;

			x.u=__REV( *(uint32_t*)(dataBuffer+0x16+0)), gyro[0]=x.f;
			x.u=__REV( *(uint32_t*)(dataBuffer+0x16+4)), gyro[1]=x.f;
			x.u=__REV( *(uint32_t*)(dataBuffer+0x16+8)), gyro[2]=x.f;

			x.u=__REV( *(uint32_t*)(dataBuffer+0x25+0)), mag[0]=x.f;
			x.u=__REV( *(uint32_t*)(dataBuffer+0x25+4)), mag[1]=x.f;
			x.u=__REV( *(uint32_t*)(dataBuffer+0x25+8)), mag[2]=x.f;
		}
	}
}

static ROM uint8_t config_data[] = // ACC GYRO MAG STATUS
  {0x40,0x20,0x00,0x64,0x80,0x20,0x00,0x64,0xC0,0x20,0x00,0x64,0xE0,0x20,0x00,0x00};

void configure_MTI_data_output( MtsspInterface* device, uint8_t * dataBuffer)
{
	volatile unsigned loops=0;
	uint16_t notificationMessageSize;
	uint16_t measurementMessageSize;

	XbusMessage cnf( XMID_GotoConfig);
	device->sendXbusMessage(&cnf);
	do
	{
		++loops;
		delay(1);
		device->readPipeStatus(notificationMessageSize, measurementMessageSize);
		if (notificationMessageSize && notificationMessageSize < DATA_BUFSIZE_BYTES)
		{
			device->readFromPipe(&dataBuffer[2], notificationMessageSize, XBUS_NOTIFICATION_PIPE);
		}

		if (measurementMessageSize && measurementMessageSize < DATA_BUFSIZE_BYTES)
		{
			device->readFromPipe(&dataBuffer[2], measurementMessageSize, XBUS_MEASUREMENT_PIPE);

		}

	}while(notificationMessageSize > 10);

	XbusMessage msg(XMID_SetOutputConfig);
	msg.m_length = sizeof(config_data);
	msg.m_data = (uint8_t *)config_data;
	device->sendXbusMessage(&msg);
}

/*!	\brief Returns the value of the DataReady line
*/
static bool checkDataReadyLine(void)
{
	return HAL_GPIO_ReadPin( IMU_PORT, IMU_DRDY ) == GPIO_PIN_SET;
}

inline void sync_data_ready(void)
{
	for( bool ready=false; ! ready; ready=checkDataReadyLine())
		delay(1);
}

static void run( void *)
{
	uint8_t dataBuffer[DATA_BUFSIZE_BYTES];
	MtsspDriverSpi SPI_driver;
	MtsspInterface IMU_interface( & SPI_driver);

	init_ports_and_reset_mti();

	sync_data_ready();

	readDataFrom_MTI( &IMU_interface, dataBuffer);

	configure_MTI_data_output( &IMU_interface, dataBuffer);

	sync_data_ready();

	readDataFrom_MTI( &IMU_interface, dataBuffer);

	XbusMessage cnf( XMID_GotoMeasurement);
	IMU_interface.sendXbusMessage( &cnf);

	for( synchronous_timer t(8); true; t.sync())
	{
		sync_data_ready();
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


