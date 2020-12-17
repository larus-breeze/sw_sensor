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
COMMON TaskHandle_t MTi_task;

extern RestrictedTask mti_driver;

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

	GPIO_InitStruct.Pin = IMU_DRDY;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	HAL_GPIO_Init(IMU_PORT, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	HAL_GPIO_WritePin( IMU_PORT, IMU_NRST, GPIO_PIN_RESET);
	delay(1);
	HAL_GPIO_WritePin( IMU_PORT, IMU_NRST, GPIO_PIN_SET);
	delay(1);
}

#define DATA_BUFSIZE_BYTES 128

typedef union{ float f; uint32_t u;} float_word;

/*!	\brief Read data from the Notification and Control pipes of the device
*/
void readDataFrom_MTI( MtsspInterface* device, uint8_t * buf)
{
	uint16_t notificationMessageSize;
	uint16_t measurementMessageSize;
	device->readPipeStatus(notificationMessageSize, measurementMessageSize);

	buf[0] = XBUS_PREAMBLE;
	buf[1] = XBUS_MASTERDEVICE;

	if (notificationMessageSize && notificationMessageSize < DATA_BUFSIZE_BYTES)
	{
		device->readFromPipe(&buf[2], notificationMessageSize, XBUS_NOTIFICATION_PIPE);
	}

	if (measurementMessageSize && measurementMessageSize < DATA_BUFSIZE_BYTES)
	{
		device->readFromPipe(&buf[2], measurementMessageSize, XBUS_MEASUREMENT_PIPE);
		if(buf[4]==0x40 && buf[0x13]==0x80 && buf[0x22]==0xC0)
		{
			float_word x;
			x.u = __REV( *(uint32_t*)(buf+0x07+0)), acc[0]=x.f;
			x.u = __REV( *(uint32_t*)(buf+0x07+4)), acc[1]=x.f;
			x.u = __REV( *(uint32_t*)(buf+0x07+8)), acc[2]=x.f;

			x.u = __REV( *(uint32_t*)(buf+0x16+0)), gyro[0]=x.f;
			x.u = __REV( *(uint32_t*)(buf+0x16+4)), gyro[1]=x.f;
			x.u = __REV( *(uint32_t*)(buf+0x16+8)), gyro[2]=x.f;

			x.u = __REV( *(uint32_t*)(buf+0x25+0)), mag[0]=x.f;
			x.u = __REV( *(uint32_t*)(buf+0x25+4)), mag[1]=x.f;
			x.u = __REV( *(uint32_t*)(buf+0x25+8)), mag[2]=x.f;
		}
	}
}

extern "C" void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(IMU_DRDY);
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if((GPIO_Pin == IMU_DRDY) && MTi_task)
	  mti_driver.notify_from_ISR( 1, eSetBits);
}

/*!	\brief Returns the value of the DataReady line
*/
//static bool checkDataReadyLine(void)
//{
//	return HAL_GPIO_ReadPin( IMU_PORT, IMU_DRDY ) == GPIO_PIN_SET;
//}

static ROM uint8_t config_data[] = // config: ACC GYRO MAG STATUS
  {0x40,0x20,0x00,0x64,0x80,0x20,0x00,0x64,0xC0,0x20,0x00,0x64,0xE0,0x20,0x00,0x00};

inline void sync_data_ready(void)
{
	xTaskNotifyStateClear(MTi_task);
	notify_take(1, 10);
//	for( bool ready=checkDataReadyLine(); ! ready; ready=checkDataReadyLine())
//		delay(2);
}

static void run( void *)
{
	acquire_privileges();
	init_ports_and_reset_mti();
	drop_privileges();

	MTi_task=xTaskGetCurrentTaskHandle();

	uint8_t buf[DATA_BUFSIZE_BYTES];
	MtsspDriverSpi SPI_driver;
	MtsspInterface IMU_interface( & SPI_driver);

	sync_data_ready();
	readDataFrom_MTI( &IMU_interface, buf);

	XbusMessage go_cnf( XMID_GotoConfig);
	IMU_interface.sendXbusMessage(&go_cnf);

	sync_data_ready();
	readDataFrom_MTI( &IMU_interface, buf);

	XbusMessage msg(XMID_SetOutputConfig);
	msg.m_length = sizeof(config_data);
	msg.m_data = (uint8_t *)config_data;
	IMU_interface.sendXbusMessage(&msg);

	sync_data_ready();
	readDataFrom_MTI( &IMU_interface, buf);

	XbusMessage cnf( XMID_GotoMeasurement);
	IMU_interface.sendXbusMessage( &cnf);

	for( synchronous_timer t(10); true; t.sync())
	{
		sync_data_ready();
		readDataFrom_MTI( &IMU_interface, buf);
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


