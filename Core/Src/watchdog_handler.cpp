/** *****************************************************************************
 * @file    	watchdog_handler.cpp
 * @brief   	initialize and service watchdog (with medium priority)
 * @author  	Dr. Klaus Schaefer
 * @copyright 	Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

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

#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"

static COMMON WWDG_HandleTypeDef WwdgHandle;

void heartbeat (void)
{
  bool set = GPIO_PIN_SET
      == HAL_GPIO_ReadPin ( LED_STATUS3_GPIO_Port, LED_STATUS3_Pin);
  HAL_GPIO_WritePin (LED_STATUS3_GPIO_Port, LED_STATUS3_Pin,
		     set ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

#if WATCHDOG_STATISTICS
COMMON volatile uint32_t watchdog_min=0xffffffff;
COMMON volatile uint32_t watchdog_max=0;
#endif

void initialize_watchdog(void)
{
  __HAL_RCC_WWDG_CLK_ENABLE();
  WwdgHandle.Instance = WWDG;
  WwdgHandle.Init.Prescaler = WWDG_PRESCALER_8;
  WwdgHandle.Init.Window = 0x60;
  WwdgHandle.Init.Counter = 127;
  WwdgHandle.Init.EWIMode=WWDG_EWI_ENABLE;

  NVIC_SetPriority ((IRQn_Type) WWDG_IRQn, WATCHDOG_ISR_PRIORITY);
  NVIC_EnableIRQ ((IRQn_Type) WWDG_IRQn);

  if (HAL_WWDG_Init (&WwdgHandle) != HAL_OK)
    Error_Handler ();
}

void watchdog_runnable (void*)
{
#if ACTIVATE_WATCHDOG
  acquire_privileges();

  initialize_watchdog();

  drop_privileges();

#endif // ACTIVATE_WATCHDOG


  uint8_t rythm = 0;
  for (synchronous_timer t (40); true;)
    {
      t.sync ();
      if( (++rythm & 0x0f) ==0)
	  HAL_GPIO_TogglePin (LED_STATUS3_GPIO_Port, LED_STATUS3_Pin);

#if ACTIVATE_WATCHDOG
#if WATCHDOG_STATISTICS
      uint32_t watchdog_actual = WwdgHandle.Instance->CR & 0x7f;
      if( watchdog_actual > watchdog_max)
	watchdog_max = watchdog_actual;
      if( watchdog_actual < watchdog_min)
	watchdog_min = watchdog_actual;
#endif
      HAL_WWDG_Refresh (&WwdgHandle);
#endif
    }
}

COMMON RestrictedTask watchdog_handler ( watchdog_runnable, "WATCHDOG", configMINIMAL_STACK_SIZE, 0, WATCHDOG_TASK_PRIORITY);

extern "C" void handle_watchdog_trigger( void);

unsigned watchdog_counter = 0;
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *handle)
{
  ++watchdog_counter;	// give time for the uSD system to record the crash
  if( watchdog_counter < 50) // then allow the watchdog to reset the uContoller
    HAL_WWDG_Refresh (handle);
}

extern "C" void WWDG_IRQHandler(void)
{
  HAL_WWDG_IRQHandler(&WwdgHandle);
  if( watchdog_counter < 2) // do this just once, watchdog_counter already incremented to 1
    handle_watchdog_trigger();
}
