/**
 * @file 	system_support.cpp
 * @brief 	helper functions for FreeRTOS
 * @author: 	Dr. Klaus Schaefer
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

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "my_assert.h"

#if configUSE_TRACE_FACILITY == 1
#include "trcConfig.h"
PRIVILEGED_DATA RecorderDataType myTraceBuffer;
#endif

extern uint32_t _s_system_ram[]; // provided by linker description file
extern uint32_t __user_data_end__[];
extern uint32_t __common_data_start__[];
extern uint32_t __common_data_end__[];

extern "C" void vPortInitMemory ();
volatile uint64_t SystemTicks;

extern "C" void initialize_RTOS_memory(void)
{
	/* zero out global data inside FreeRTOS system memory and inside user global memory */
	for(volatile uint32_t *ptr = _s_system_ram; ptr < __user_data_end__; ++ptr)
		*ptr = 0;

	/* zero out COMMON data */
	for(volatile uint32_t *ptr = __common_data_start__; ptr < __common_data_end__; ++ptr)
		*ptr = 0;

	vPortInitMemory ();

#if configUSE_TRACE_FACILITY == 1
  vTraceSetRecorderDataBuffer(&myTraceBuffer);
  vTraceEnable(TRC_INIT);
#endif
}

volatile uint32_t idle_counter;

extern "C" void vApplicationIdleHook( void)
{
	++idle_counter;
	__WFI();
}

extern "C" __weak void Systick_Callback( uint64_t ticks)
{
}

/********************************************************************//**
 * @brief Tick Hook: Callback for system Tick ISR
 *
 * do SystemTicks timekeeping
 *********************************************************************/
extern "C" void vApplicationTickHook( void)
{
  ++SystemTicks;
  HAL_IncTick();
  Systick_Callback( SystemTicks);
}

uint64_t getTime_usec_privileged(void)
{
  uint64_t time;
  uint64_t present_systick;
  uint64_t reload = (* (uint32_t *)0xe000e014) + 1;

  do
  {
	  __DSB();
	  time= __LDREXW( (uint32_t*)&SystemTicks);
	  present_systick=(uint64_t) ( * (uint32_t *)0xe000e018);
	  __DSB();
  }
  while( __STREXW( (uint32_t)time, (uint32_t*)&SystemTicks) != 0)

  ; // milliseconds -> microseconds
  time *= 1000;

  present_systick = reload - present_systick; // because it's a down-counter
  present_systick *= 1000; // millisecs -> microsecs
  present_systick /= reload;

  return time + present_systick;
}

extern "C" BaseType_t xPortRaisePrivilege( void );

typedef int ( *FPTR)( void *); // declare void* -> int function pointer
int call_function_privileged( void * parameters, FPTR function)
{
  portBASE_TYPE running_privileged = xPortRaisePrivilege();
  int retval = function( parameters );
  if( ! running_privileged)
    portSWITCH_TO_USER_MODE(); // go protected again
  return retval;
}

static int getTime_usec_helper(void *parameters)
{
  *(uint64_t *)parameters = getTime_usec_privileged();
  return 0; // no meaningful return value here
}

extern "C" void HAL_Delay(uint32_t delay)
{
	MPU_vTaskDelay( delay);
}

uint64_t getTime_usec(void)
{
  uint64_t retv;
  (void)call_function_privileged( &retv, getTime_usec_helper);
  return retv;
}

/**
 * @brief  This function handles FreeRTOS's Stack Overflow exception.
 */
void
vApplicationStackOverflowHook(void)
{
  ASSERT( 0);
}
