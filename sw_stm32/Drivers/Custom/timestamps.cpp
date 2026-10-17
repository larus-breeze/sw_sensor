/***********************************************************************//**
 * @file		timestamps.cpp
 * @brief		precise acquisition times of the observations (#263)
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

// 48 bit timestamp counter (TIM4 + TIM5, 84 MHz) and input captures of the
// IMU data-ready line and the GNSS time pulse, see documentation/timestamps.md

#include "system_configuration.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "embedded_memory.h"
#include "timestamps.h"

COMMON timestamps_t timestamps;

//! TIM5 counts a TIM4 overflow a few timer clocks after it happened: while
//! TIM4 is that close to its overflow, the high part may not have been
//! counted yet
#define SYNC_MARGIN_TICKS 16

static bool timestamps_initialized = false;

void timestamps_init (void)
{
  if (timestamps_initialized)
    return;

  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  // TIM5: counts TIM4 overflows (external clock mode 1, trigger ITR2 = TIM4 TRGO)
  TIM5->CR1 = 0;
  TIM5->PSC = 0;
  TIM5->ARR = 0xffffffff;
  TIM5->SMCR = TIM_SMCR_TS_1 | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
  TIM5->CNT = 0;
  TIM5->CR1 = TIM_CR1_CEN;

  // TIM4: free running at the timer clock, update event -> TRGO
  TIM4->CR1 = 0;
  TIM4->PSC = 0;
  TIM4->ARR = 0xffff;
  TIM4->CR2 = TIM_CR2_MMS_1;
  TIM4->CCMR1 = TIM_CCMR1_CC1S_0; // CC1 = input capture on TI1 (PD12)
  TIM4->CCMR2 = TIM_CCMR2_CC3S_0; // CC3 = input capture on TI3 (PD14)
  TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC3E; // rising edges
  TIM4->DIER = TIM_DIER_CC3IE; // time pulse; the IMU capture is read in the data-ready EXTI interrupt
  TIM4->SR = 0;
  TIM4->CNT = 0;
  TIM4->CR1 = TIM_CR1_CEN;

  // PD14: GNSS time pulse input, pulled down while not connected on the board
  GPIO_InitTypeDef GPIO_InitStruct = { 0, 0, 0, 0, 0 };
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init (GPIOD, &GPIO_InitStruct);

  HAL_NVIC_SetPriority (TIM4_IRQn, STANDARD_ISR_PRIORITY, 0);
  HAL_NVIC_EnableIRQ (TIM4_IRQn);

  timestamps_initialized = true;
}

uint64_t timestamp_now_privileged (void)
{
  while (true)
    {
      uint32_t low = TIM4->CNT;
      if (low < SYNC_MARGIN_TICKS)
	continue; // TIM5 may not have counted this overflow yet
      uint32_t high = TIM5->CNT;
      if (TIM4->CNT < low)
	continue; // TIM4 has wrapped meanwhile, read again
      return ((uint64_t) high << 16) | low;
    }
}

//! extend a 16 bit capture value, read within one TIM4 period after the edge
static uint64_t extend_capture (uint32_t captured)
{
  uint64_t now = timestamp_now_privileged ();
  uint64_t time = (now & ~(uint64_t) 0xffff) | captured;
  if (captured > (now & 0xffff))
    time -= 0x10000; // captured before the latest overflow
  return time;
}

uint64_t timestamp_IMU_data_ready_privileged (void)
{
  if (! timestamps_initialized)
    return 0;
  if (TIM4->SR & TIM_SR_CC1IF)
    return extend_capture (TIM4->CCR1); // reading CCR1 clears CC1IF
  return timestamp_now_privileged (); // no capture: interrupt entry time as fallback
}

//! TIM4 interrupt: GNSS time pulse capture
extern "C" void TIM4_IRQHandler (void)
{
  if (TIM4->SR & TIM_SR_CC3IF)
    {
      uint64_t time = extend_capture (TIM4->CCR3); // reading CCR3 clears CC3IF
      timestamps.GNSS_timepulse = (uint32_t) time;
    }
  TIM4->SR = ~(TIM_SR_CC1OF | TIM_SR_CC3OF); // over-capture flags are of no interest
}
