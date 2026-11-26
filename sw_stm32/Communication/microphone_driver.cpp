/***********************************************************************//**
 * @file		microphone_driver.cpp
 * @brief		ICS-43434 I2S microphone on I2S3: ENL and noise band levels
 * @license 		This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/

// Wiring, clocking and data flow: documentation/microphone_ENL.md

#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "common.h"
#include "my_assert.h"
#include "microphone_driver.h"

#if RUN_MICROPHONE

// PLLI2S: 2 MHz (16 MHz HSE / PLLM 8) * 128 = 256 MHz VCO (100..432 MHz),
// / 5 = 51.2 MHz. fs = 51.2 MHz / (64 * 2 * 50) = 8 kHz, SCK = 64 * fs =
// 512 kHz, which puts the ICS-43434 into its low-power mode (SCK 460..870 kHz)
#define MIC_PLLI2SN		128
#define MIC_PLLI2SR		5
#define MIC_I2SDIV		50
#define MIC_I2S_CLOCK_HZ	51200000u
static_assert (MIC_I2S_CLOCK_HZ / (64 * 2 * MIC_I2SDIV) == (unsigned) MIC_SAMPLE_RATE_HZ,
	       "I2S divider does not match the analysis sample rate");

#define MIC_BLOCK_FRAMES	128	// 16 ms per DMA half: room for higher-priority tasks
#define MIC_HALFWORDS_PER_FRAME	4	// two 32-bit slots, read as 16-bit halves
#define MIC_DMA_HALFWORDS	(2 * MIC_BLOCK_FRAMES * MIC_HALFWORDS_PER_FRAME)
#define MIC_DMA_BYTES		(MIC_DMA_HALFWORDS * sizeof(uint16_t))
#define MIC_TIMEOUT_MS		100
#define MIC_DETECTION_RECORDS	8	// ~2.75 s after start-up incl. settling
#define MIC_LOST_BLOCKS_AFTER_MS 10000	// start-up (task initializations) may delay the MIC task: not counted

enum
{
  MIC_NOTIFY_HALF	= 1,
  MIC_NOTIFY_COMPLETE	= 2,
  MIC_NOTIFY_ERROR	= 4
};

#define DMA_STREAM0_FLAGS (DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | DMA_LISR_TCIF0)

// in RAM, not CCM: DMA has no access to the CCM
static uint16_t __ALIGNED(MIC_DMA_BYTES) mic_dma_buffer[MIC_DMA_HALFWORDS];

COMMON Queue <microphone_record_t> microphone_record_queue (2);
COMMON microphone_record_t microphone_data;
// odd while microphone_data is written: the MIC task preempts lower-priority readers
COMMON static volatile uint32_t microphone_data_count;
// in COMMON: too big for the task's 1 KB stack
COMMON static microphone_analysis_t analysis;
COMMON static volatile microphone_state_t microphone_state = MICROPHONE_DETECTING;

extern RestrictedTask microphone_task;

static void mic_hardware_init (void)
{
  RCC_PeriphCLKInitTypeDef clock = { 0 };
  clock.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  clock.PLLI2S.PLLI2SN = MIC_PLLI2SN;
  clock.PLLI2S.PLLI2SR = MIC_PLLI2SR;
  if (HAL_RCCEx_PeriphCLKConfig (&clock) != HAL_OK)
    ASSERT(0);
  // a wrong rate shifts every band (first version: 16 kHz, HSE taken for 8 MHz)
  if (HAL_RCCEx_GetPeriphCLKFreq (RCC_PERIPHCLK_I2S) != MIC_I2S_CLOCK_HZ)
    ASSERT(0);

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SPI3_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  // PA15 = I2S3_WS, PB3 = I2S3_CK (SWO, so no SWV trace), PB5 = I2S3_SD.
  // These are JTAG pins after reset, switching them to AF6 releases them,
  // SWD (PA13/PA14) keeps working.
  GPIO_InitTypeDef gpio = { 0 };
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF6_SPI3;
  gpio.Pin = GPIO_PIN_15;
  HAL_GPIO_Init (GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_3;
  HAL_GPIO_Init (GPIOB, &gpio);
  // SD is tri-stated during the other channel's slot: pull it to 0
  gpio.Pin = GPIO_PIN_5;
  gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init (GPIOB, &gpio);

  HAL_NVIC_SetPriority (DMA1_Stream0_IRQn, STANDARD_ISR_PRIORITY, 0);
  HAL_NVIC_EnableIRQ (DMA1_Stream0_IRQn);
}

//! (re)start I2S3 and DMA1 stream 0 channel 0 (SPI3_RX), circular
//! peripheral registers are reachable unprivileged, so this works after drop_privileges()
static void mic_start (void)
{
  SPI3->I2SCFGR = 0;
  SPI3->CR2 = 0;
  DMA1_Stream0->CR &= ~DMA_SxCR_EN;
  while (DMA1_Stream0->CR & DMA_SxCR_EN)
    ;
  (void) SPI3->DR; // clears RXNE and, followed by the SR read, OVR
  (void) SPI3->SR;
  DMA1->LIFCR = DMA_STREAM0_FLAGS;

  DMA1_Stream0->PAR = (uint32_t) &SPI3->DR;
  DMA1_Stream0->M0AR = (uint32_t) mic_dma_buffer;
  DMA1_Stream0->NDTR = MIC_DMA_HALFWORDS;
  DMA1_Stream0->FCR = 0; // direct mode
  DMA1_Stream0->CR = DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0
      | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE | DMA_SxCR_TEIE; // CHSEL = 0
  DMA1_Stream0->CR |= DMA_SxCR_EN;

  // ICS-43434: Philips I2S, 24 bit data in 32 bit slots, data valid on the rising SCK edge
  SPI3->I2SPR = MIC_I2SDIV; // ODD = 0, MCKOE = 0
  SPI3->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_0 | SPI_I2SCFGR_I2SCFG_1 // master receive
      | SPI_I2SCFGR_DATLEN_0 | SPI_I2SCFGR_CHLEN;
  SPI3->CR2 = SPI_CR2_RXDMAEN;
  SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE; // master: the first slot received is the left channel
}

//! no microphone: release the pins and clocks, I2S stays off until the next start-up
static void mic_stop (void)
{
  SPI3->I2SCFGR = 0;
  SPI3->CR2 = 0;
  DMA1_Stream0->CR &= ~DMA_SxCR_EN;
  while (DMA1_Stream0->CR & DMA_SxCR_EN)
    ;
  DMA1->LIFCR = DMA_STREAM0_FLAGS;

  acquire_privileges ();
  HAL_NVIC_DisableIRQ (DMA1_Stream0_IRQn);
  __HAL_RCC_SPI3_CLK_DISABLE();
  __HAL_RCC_PLLI2S_DISABLE();
  drop_privileges();

  // high impedance: on hardware without a microphone these pins may be wired otherwise
  GPIO_InitTypeDef gpio = { 0 };
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  gpio.Pin = GPIO_PIN_15;
  HAL_GPIO_Init (GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_3 | GPIO_PIN_5;
  HAL_GPIO_Init (GPIOB, &gpio);
}

extern "C" void DMA1_Stream0_IRQHandler (void)
{
  uint32_t flags = DMA1->LISR & DMA_STREAM0_FLAGS;
  DMA1->LIFCR = flags;

  uint32_t notification = 0;
  if (flags & DMA_LISR_HTIF0)
    notification |= MIC_NOTIFY_HALF;
  if (flags & DMA_LISR_TCIF0)
    notification |= MIC_NOTIFY_COMPLETE;
  if (flags & DMA_LISR_TEIF0)
    notification |= MIC_NOTIFY_ERROR;
  if (notification)
    microphone_task.notify_from_ISR (notification, eSetBits);
}

static inline int32_t mic_sample (const uint16_t *frame, unsigned channel)
{
  // the MSB half of a 32 bit slot arrives first
  return (int32_t) (((uint32_t) frame[2 * channel] << 16) | frame[2 * channel + 1]);
}

static void microphone_runnable (void*)
{
  mic_hardware_init ();
  mic_start ();
  drop_privileges();

  unsigned lost_blocks = 0;
  unsigned detection_records = 0;

  while (true)
    {
      uint32_t notification = 0;
      bool notified = Task::notify_wait (0, 0xffffffff, notification, MIC_TIMEOUT_MS);

      if ( ! notified || (notification & MIC_NOTIFY_ERROR) || (SPI3->SR & SPI_SR_OVR))
	{
	  mic_start ();
	  analysis.reset ();
	  continue;
	}

      if ((notification & (MIC_NOTIFY_HALF | MIC_NOTIFY_COMPLETE)) == (MIC_NOTIFY_HALF | MIC_NOTIFY_COMPLETE))
	{
	  // too late for one half, take the latest
	  if (xTaskGetTickCount () >= pdMS_TO_TICKS (MIC_LOST_BLOCKS_AFTER_MS))
	    ++lost_blocks;
	}

      // the half the DMA is not writing to is the latest complete one
      unsigned half = (DMA1_Stream0->NDTR > MIC_DMA_HALFWORDS / 2) ? 1 : 0;
      const uint16_t *block = mic_dma_buffer + half * (MIC_DMA_HALFWORDS / 2);

      // the slot the microphone uses depends on its SEL pin, the other one reads 0
      uint32_t activity[2] = { 0, 0 };
      for (unsigned i = 0; i < MIC_BLOCK_FRAMES; ++i)
	for (unsigned channel = 0; channel < 2; ++channel)
	  activity[channel] |= (uint32_t) mic_sample (block + i * MIC_HALFWORDS_PER_FRAME, channel);
      unsigned channel = (activity[0] == 0 && activity[1] != 0) ? 1 : 0;

      for (unsigned i = 0; i < MIC_BLOCK_FRAMES; ++i)
	if (analysis.update (mic_sample (block + i * MIC_HALFWORDS_PER_FRAME, channel)))
	  {
	    microphone_record_t record = analysis.get_record ();

	    // Once detected, every record is logged: a microphone failing
	    // later must show up as ENL 000 (IGC: unserviceable).
	    if (microphone_state == MICROPHONE_DETECTING)
	      {
		if ((record.status & MIC_STATUS_NO_SIGNAL) == 0)
		  microphone_state = MICROPHONE_PRESENT;
		else if (++detection_records >= MIC_DETECTION_RECORDS)
		  {
		    microphone_state = MICROPHONE_ABSENT;
		    mic_stop ();
		    while (true)
		      suspend ();
		  }
		else
		  continue;
	      }

	    record.status |= (lost_blocks > 0xffff ? 0xffff : lost_blocks) << MIC_STATUS_LOST_BLOCKS_SHIFT;
	    ++microphone_data_count;
	    __asm volatile ("" ::: "memory");
	    microphone_data = record;
	    __asm volatile ("" ::: "memory");
	    ++microphone_data_count;
	    microphone_record_queue.send (record, 0);
	  }
    }
}

bool microphone_get_latest (microphone_record_t &record)
{
  for (unsigned attempt = 0; attempt < 3; ++attempt)
    {
      uint32_t before = microphone_data_count;
      __asm volatile ("" ::: "memory");
      record = microphone_data;
      __asm volatile ("" ::: "memory");
      if (((before & 1) == 0) && (before == microphone_data_count))
	return before != 0;
    }
  return false;
}

microphone_state_t microphone_get_state (void)
{
  return microphone_state;
}

#define MIC_STACK_WORDS 256
// static: the FreeRTOS heap (CCM) may have no free block with the
// stack's size alignment the MPU needs
static uint32_t __ALIGNED(MIC_STACK_WORDS * sizeof(uint32_t)) mic_stack[MIC_STACK_WORDS];

static ROM TaskParameters_t p =
  {
    microphone_runnable,
    "MIC",
    MIC_STACK_WORDS,
    0,
    (MICROPHONE_PRIORITY) | portPRIVILEGE_BIT, // privileged for the hardware setup
    mic_stack,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { mic_dma_buffer, MIC_DMA_BYTES, portMPU_REGION_READ_ONLY },
      { 0, 0, 0 }
    }
  };

COMMON RestrictedTask microphone_task (p);

#else

bool microphone_get_latest (microphone_record_t &)
{
  return false;
}

microphone_state_t microphone_get_state (void)
{
  return MICROPHONE_NOT_BUILT;
}

#endif
