#include "FreeRTOS_wrapper.h"

uint64_t getTime_usec_privileged(void);

void busy_wait_until( volatile uint32_t microseconds)
{
	microseconds -=3; // tune :-)
	uint64_t now = getTime_usec_privileged();
	while( (getTime_usec_privileged() - now) < microseconds)
		/* spinlock loop */;
}

void busy_wait( volatile uint32_t microseconds)
{
	microseconds = (microseconds * 382) >> 4;
	while( microseconds--)
		/* spinlock loop */;
}

volatile uint64_t time;

void runnable1( void *)
{
	while( true)
	{
		delay(13);
//		__disable_irq();
		time = getTime_usec_privileged();
		busy_wait( 10);
		time = getTime_usec_privileged()-time;
//		__enable_irq();
	}
}

Task task1( runnable1, "TSK1", configMINIMAL_STACK_SIZE, 0, STANDARD_TASK_PRIORITY+4);
