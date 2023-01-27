#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"

#if INJECT_ERROR_NUMBER

COMMON unsigned fault_type = INJECT_ERROR_NUMBER;
COMMON volatile float a,b,c;

volatile void recursion(void)
{
  recursion();
}

void runnable( void * p_fault_type)
{
  delay( 10000);
  switch( *(unsigned *)p_fault_type)
  {
    case 1:
	// try bad memory access
	*(unsigned *) 0x2000000 = 13;
      break;
    case 2:
      // leave task runnable
      return;
      break;
    case 3:
      // execute illegal code
      void vPortEndScheduler( void );
      vPortEndScheduler();
      break;
    case 4:
      // divide by zero
	a = 12345.6;
	b = 0.0f;
	c = a / b;
      break;
    case 5:
      // stack overflow
      recursion();
      break;
    case 6:
	ASSERT( 0);
      return;
      break;
    case 7:
      while( true)
	/* tease our watchdog */;
      break;
    default:
      while( true) // defensively: go sleeping
	suspend();
      break;
  }
  while( true) // defensively: go sleeping
    suspend();
}

RestrictedTask inject_fault( runnable, "FAULT", configMINIMAL_STACK_SIZE, (void *)&fault_type, STANDARD_TASK_PRIORITY + 4);

#endif

