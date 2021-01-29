#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#if configUSE_TRACE_FACILITY == 1
#include "trcConfig.h"
PRIVILEGED_DATA RecorderDataType myTraceBuffer;
#endif

extern uint32_t _s_system_ram[]; // provided by linker description file
extern uint32_t __user_data_end__[];

extern "C" void vPortInitMemory ();
volatile uint64_t SystemTicks;

extern "C" void initialize_RTOS_memory(void)
{
	/* zero out global data inside FreeRTOS system memory and inside user global memory */
	for(volatile uint32_t *ptr = _s_system_ram; ptr < __user_data_end__; ++ptr)
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
//	*(uint32_t *)0x40022018=0x020000;
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
  __asm volatile ( "bkpt 0" );
}
