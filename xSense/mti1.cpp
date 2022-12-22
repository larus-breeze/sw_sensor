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
#include "communicator.h"

#if RUN_MTi_1_MODULE

// fine-tuned MTi timing parameters
#define LONGEST_WAIT_4_MTI_MS 400
#define PLANNED_DELAY_4_MTI_MS 20
#define DAQ_LOOP_WAIT_4_MTI_MS 15

// used GPIO pins
#define IMU_PSEL0  GPIO_PIN_10
#define IMU_PSEL1  GPIO_PIN_11
#define IMU_DRDY   GPIO_PIN_12
#define IMU_NRST   GPIO_PIN_13
#define IMU_PORT   GPIOD

COMMON Semaphore MTi_ready; //!< ISR -> task synchronizing semaphore

void sync_communicator (void);

extern RestrictedTask mti_driver;

#if TRACE_ISR == 1
COMMON traceString chn;
COMMON traceHandle EXTI15_10_Handle;
#endif

static inline void
init_ports_and_reset_mti (void) // GPIO stuff
{
#if TRACE_ISR == 1
  EXTI15_10_Handle = xTraceSetISRProperties("EXTI15_10", 15);
#endif

  GPIO_InitTypeDef GPIO_InitStruct ={ 0 };

  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Pin = IMU_PSEL0 | IMU_PSEL1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init (IMU_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin ( IMU_PORT, IMU_PSEL0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin ( IMU_PORT, IMU_PSEL1, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = IMU_NRST;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init (IMU_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IMU_DRDY;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init (IMU_PORT, &GPIO_InitStruct);

  HAL_NVIC_SetPriority (EXTI15_10_IRQn, STANDARD_ISR_PRIORITY, 0);
  HAL_NVIC_EnableIRQ (EXTI15_10_IRQn);

  HAL_GPIO_WritePin ( IMU_PORT, IMU_NRST, GPIO_PIN_RESET);
  delay (PLANNED_DELAY_4_MTI_MS);
  HAL_GPIO_WritePin ( IMU_PORT, IMU_NRST, GPIO_PIN_SET);
  delay (1);
}

#define DATA_BUFSIZE_BYTES 128

typedef union
{
  float f;
  uint32_t u;
} float_word;

/*!	\brief Read data from the Notification and Control pipes of the device
 */
void
readDataFrom_MTI (MtsspInterface *device, uint8_t *buf)
{
  uint16_t notificationMessageSize;
  uint16_t measurementMessageSize;
  device->readPipeStatus (notificationMessageSize, measurementMessageSize);

  buf[0] = XBUS_PREAMBLE;
  buf[1] = XBUS_MASTERDEVICE;

  if (notificationMessageSize && notificationMessageSize < DATA_BUFSIZE_BYTES)
    {
      device->readFromPipe (&buf[2], notificationMessageSize,
			    XBUS_NOTIFICATION_PIPE);
    }

  if (measurementMessageSize && measurementMessageSize < DATA_BUFSIZE_BYTES)
    {
      device->readFromPipe (&buf[2], measurementMessageSize, XBUS_MEASUREMENT_PIPE);
      if (buf[4] == 0x40 && buf[0x13] == 0x80 && buf[0x22] == 0xC0)
	{
	  float_word x;
	  x.u = __REV (*(uint32_t*) (buf + 0x07 + 0));
	  output_data.m.acc[0] = x.f;
	  x.u = __REV (*(uint32_t*) (buf + 0x07 + 4));
	  output_data.m.acc[1] = x.f;
	  x.u = __REV (*(uint32_t*) (buf + 0x07 + 8));
	  output_data.m.acc[2] = x.f;

	  x.u = __REV (*(uint32_t*) (buf + 0x16 + 0));
	  output_data.m.gyro[0] = isnormal(x.f) ? x.f : 0.0f;
	  x.u = __REV (*(uint32_t*) (buf + 0x16 + 4));
	  output_data.m.gyro[1] = isnormal(x.f) ? x.f : 0.0f;
	  x.u = __REV (*(uint32_t*) (buf + 0x16 + 8));
	  output_data.m.gyro[2] = isnormal(x.f) ? x.f : 0.0f;

	  x.u = __REV (*(uint32_t*) (buf + 0x25 + 0));
	  output_data.m.mag[0] = x.f;
	  x.u = __REV (*(uint32_t*) (buf + 0x25 + 4));
	  output_data.m.mag[1] = x.f;
	  x.u = __REV (*(uint32_t*) (buf + 0x25 + 8));
	  output_data.m.mag[2] = x.f;
	}
    }
}

/**
 * @brief EXTI15_10 interrupt handler
 */
extern "C" void EXTI15_10_IRQHandler (void)
{
#if TRACE_ISR == 1
  vTraceStoreISRBegin(EXTI15_10_Handle);
#endif

  HAL_GPIO_EXTI_IRQHandler (IMU_DRDY);

#if TRACE_ISR == 1
  vTraceStoreISREnd(EXTI15_10_Handle);
#endif
}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IMU_DRDY)
    MTi_ready.signal_from_ISR ();
}

/*!	\brief Returns the value of the DataReady line
 */
static inline bool checkDataReadyLine (void)
{
  return HAL_GPIO_ReadPin ( IMU_PORT, IMU_DRDY) == GPIO_PIN_SET;
}

static ROM uint8_t config_data[] = // config: ACC GYRO MAG STATUS
      { 0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64, 0xC0, 0x20, 0x00, 0x64, 0xE0, 0x20, 0x00, 0x00 };
// "wrong" config:
//{0x80,0x30,0x00,0x00,0x40,0x20,0x00,0x64,0x80,0x20,0x00,0x64,0xC0,0x20,0x00,0x64,0xE0,0x20,0x00,0x00};

static bool check_for_correct_configuration (uint8_t *data)
{
  unsigned counter = 0;
  for (const uint8_t *my_config = config_data; *my_config == *data; ++counter)
    {
      ++my_config;
      ++data;
    }
  return counter == sizeof(config_data) - 2;
}

static void run (void*)
{
#if TRACE_ISR == 1
	chn = xTraceRegisterString("MTi-ISR");
#endif
restart:

  acquire_privileges();
  init_ports_and_reset_mti ();
  drop_privileges();

  uint8_t buf[DATA_BUFSIZE_BYTES];
  MtsspDriverSpi SPI_driver;
  MtsspInterface IMU_interface (&SPI_driver);

  delay (LONGEST_WAIT_4_MTI_MS); // MTi 1 typically needs 168ms to reset itself

  if( false == checkDataReadyLine())
	goto restart;

  readDataFrom_MTI (&IMU_interface, buf);

  delay (PLANNED_DELAY_4_MTI_MS);

  while (true) // until configuration has been set successfully
    {
      XbusMessage read_cnf (XMID_ReqOutputConfig);
      IMU_interface.sendXbusMessage (&read_cnf);

      if( false == MTi_ready.wait (LONGEST_WAIT_4_MTI_MS))
	goto restart;

      readDataFrom_MTI (&IMU_interface, buf);

      if (check_for_correct_configuration (buf + 4)) // skip header -> + 4
	{
	  update_system_state_set (MTI_SENSOR_AVAILABE);
	  break; // now correct configuration has been confirmed
	}

      XbusMessage go_cnf (XMID_GotoConfig);
      IMU_interface.sendXbusMessage (&go_cnf);

      delay (PLANNED_DELAY_4_MTI_MS);

      XbusMessage msg (XMID_SetOutputConfig);
      msg.m_length = sizeof(config_data);
      msg.m_data = (uint8_t*) config_data;
      IMU_interface.sendXbusMessage (&msg);

      if( false == MTi_ready.wait (LONGEST_WAIT_4_MTI_MS))
	goto restart;

      readDataFrom_MTI (&IMU_interface, buf);
      if (check_for_correct_configuration (buf + 2))
	{
	  update_system_state_set (MTI_SENSOR_AVAILABE);
	  break; // now correct configuration has been confirmed
	}
    }

  XbusMessage cnf (XMID_GotoMeasurement);
  IMU_interface.sendXbusMessage (&cnf);

  if( false == MTi_ready.wait (LONGEST_WAIT_4_MTI_MS))
	goto restart;

  readDataFrom_MTI (&IMU_interface, buf);

  // the *second* DAC loop has been observed taking 35 ms
  // so: do some dummy wait + read loops here
  for( unsigned i = 0; i < 5; ++i)
    {
      if( false == MTi_ready.wait (LONGEST_WAIT_4_MTI_MS))
	  goto restart;
      readDataFrom_MTI (&IMU_interface, buf);
    }

  while (true)
    {
      if( false == MTi_ready.wait (DAQ_LOOP_WAIT_4_MTI_MS))
	goto restart;

      readDataFrom_MTI (&IMU_interface, buf);

      sync_communicator (); // trigger computations @ 100Hz
    }
}

#define STACKSIZE 256

static uint32_t __ALIGNED(STACKSIZE*4) stack_buffer[STACKSIZE];

static ROM TaskParameters_t p =
  { run, "IMU",
  STACKSIZE, 0,
  MTI_PRIORITY, stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 },
      { 0, 0, 0 } } };

RestrictedTask mti_driver (p);

#endif
