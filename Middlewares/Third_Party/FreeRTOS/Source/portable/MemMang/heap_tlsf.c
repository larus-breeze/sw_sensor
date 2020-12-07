#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"

//#define TLSF_DUMP 1
#if TLSF_DUMP
#include "semihosting.h"
#include "Trace.h"
#endif

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "tlsf.h"

tlsf_t * __attribute__ ((section ("user_data"))) the_tlsf;

void vPortInitMemory(void)
{
  the_tlsf = tlsf_create_with_pool( &__FreeRTOS_heap_begin__, &__FreeRTOS_heap_end__ - &__FreeRTOS_heap_begin__);
#if DUMP
  trace_printf ("Memory Pool: 0x%08X-0x%08X\n", &__FreeRTOS_heap_begin__, &__FreeRTOS_heap_end__);
#endif
}

void * pvPortMalloc(size_t xWantedSize)
{
   vTaskSuspendAll();
   void * res = tlsf_malloc( the_tlsf, xWantedSize);
   xTaskResumeAll();
#if DUMP
   trace_printf ("Alloc: 0x%08X-0x%08X\n", res, res + xWantedSize -1);
#endif
   return res;
}

void * pvPortMallocAlignedMemory(size_t xWantedSize, size_t alignment)
{
   vTaskSuspendAll();
   void * res = tlsf_memalign( the_tlsf, alignment, xWantedSize);
   xTaskResumeAll();
#if DUMP
   trace_printf ("Alloc: 0x%08X-0x%08X aligned 0x%08X\n", res, res + xWantedSize -1, alignment);
#endif
   return res;
}

void vPortFree(void *pv)
{
   vTaskSuspendAll();
   tlsf_free( the_tlsf, pv);
   xTaskResumeAll();
}

void * pvPortRealloc(void *pv, size_t xWantedSize)
{
   vTaskSuspendAll();
   void * res = tlsf_realloc(the_tlsf, pv, xWantedSize);
   xTaskResumeAll();
   return res;
}
