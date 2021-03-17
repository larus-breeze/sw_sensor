/** *****************************************************************************
 * @file    emergency.c
 * @author  Dr. Klaus Schaefer Hochschule Darmstadt schaefer.eit.h-da.de
 * @brief   Error handling routines
 ******************************************************************************/

#include <stdint.h>
#include "my_assert.h"
#include "stm32f407xx.h"

// "core dump" memory
volatile unsigned int stacked_r0;
volatile unsigned int stacked_r1;
volatile unsigned int stacked_r2;
volatile unsigned int stacked_r3;
volatile unsigned int stacked_r12;
volatile unsigned int stacked_lr;
volatile unsigned int stacked_pc;
volatile unsigned int stacked_psr;

uint32_t Bus_Fault_Address;
uint32_t Bad_Memory_Address;
uint32_t Fault_status;
uint32_t Bad_Instruction_Address;
uint8_t  Bus_Fault_Status;

/**
 * @brief  This function handles exceptions triggered by bad parameters to lib functions
 */
void
assert_failed(uint8_t* file, uint32_t line)
{
  __asm volatile ( "bkpt 0" );
}

void FPU_IRQHandler( void)
{
  asm("bkpt 0");
}

/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void
NMI_Handler(void)
{
  __asm volatile ( "bkpt 0" );
}

void
pop_registers_from_fault_stack(volatile unsigned int * hardfault_args)
{
  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  /* Inspect stacked_pc to locate the offending instruction. */
  __asm volatile ( "bkpt 0" );
  __asm volatile ( "bx	lr" );

}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void
HardFault_Handler(void)
{
  __asm volatile
  (
      " mov r5, #0                                                     \n"
      " tst lr, #4                                                     \n"
      " ite eq                                                         \n"
      " mrseq r0, msp                                                  \n"
      " mrsne r0, psp                                                  \n"
      " ldr r1, [r0, #24]                                              \n"
      " ldr r2, handler3_address_const                                 \n"
      " bx r2                                                          \n"
      " handler3_address_const: .word pop_registers_from_fault_stack   \n"
  );
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void
MemManage_Handler(void)
{
	  Bad_Memory_Address = *(int*) 0xe000ed34;
	  Fault_status    = *(uint8_t*) 0xe000ed28;
  // If you are stranded here:
  // Check Bad_Memory_Address and Bad_Instruction_Address !
  __asm volatile
  (
      " tst lr, #4                      		\n"
      " ite eq                          		\n"
      " mrseq r0, msp                   		\n"
      " mrsne r0, psp                   		\n"
      " ldr r1, [r0, #24]                    	\n"
      " ldr r2, instruction_address_const  		\n"
      " str r1, [r2]                    		\n"
      " bkpt	0			                	\n"
      " bx lr                               	\n"
      " instruction_address_const: .word Bad_Instruction_Address \n"
  );
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void
BusFault_Handler(void)
{
	Bus_Fault_Address=*(uint32_t *)0xe000ed38;
	Bus_Fault_Status =*(uint8_t *)0xe000ed29;
  __asm volatile
  (
      " mov r5, #2                                                     \n"
      " tst lr, #4                                                     \n"
      " ite eq                                                         \n"
      " mrseq r0, msp                                                  \n"
      " mrsne r0, psp                                                  \n"
      " ldr r1, [r0, #24]                                              \n"
      " ldr r2, handler3b_address_const                                \n"
      " bx r2                                                          \n"
      " handler3b_address_const: .word pop_registers_from_fault_stack  \n"
  );
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void __attribute__(( naked )) UsageFault_Handler(void)
{
  __asm volatile
  (
//	  " bkpt 0                                                         \n"
//	  " bx  lr		                                                   \n"
      " mov r5, #3                                                     \n"
      " tst lr, #4                                                     \n"
      " ite eq                                                         \n"
      " mrseq r0, msp                                                  \n"
      " mrsne r0, psp                                                  \n"
      " ldr r1, [r0, #24]                                              \n"
      " ldr r2, handler3u_address_const                                \n"
      " bx r2                                                          \n"
      " handler3u_address_const: .word pop_registers_from_fault_stack  \n"
  );
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

#if 0
void Default_Handler( void)
{
  uint32_t active_vector = *(NVIC->IABR);
  asm("bkpt 0");
}
#endif
/**
 * @brief  This function handles FreeRTOS's Stack Overflow exception.
 */
void
vApplicationStackOverflowHook(void)
{
  __asm volatile ( "bkpt 0" );
}

/**
 * @brief  This function handles FreeRTOS's out of memory exception.
 */
void
vApplicationMallocFailedHook(void)
{
  __asm volatile ( "bkpt 0" );
}

void vTaskSuspend(uint32_t);

void vApplicationReturnFromTaskProcedureHook( void)
{
	ASSERT(0);
	vTaskSuspend(0);
}

void abort( void)
{
	   while(1)
	  __asm volatile( "bkpt 0");
}

#define SHORTCUT( fkt_name) \
void fkt_name(void) { while(1) {__asm volatile ( "bkpt 0" );} }

//SHORTCUT( _sbrk)
SHORTCUT( _read)
SHORTCUT( _write)
SHORTCUT( __cxa_pure_virtual)
SHORTCUT( __wrap___aeabi_unwind_cpp_pr0)
