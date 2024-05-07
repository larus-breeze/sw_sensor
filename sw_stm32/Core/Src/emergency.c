/** *****************************************************************************
 * @file    	emergency.c
 * @brief   	Error handling routines
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

#include <stdint.h>
#include "my_assert.h"
#include "stm32f407xx.h"
#include "emergency.h"
#include "embedded_memory.h"

void sync_logger(void );

COMMON register_dump_t register_dump;

/**
 * @brief  This function handles exceptions triggered by bad parameters to lib functions
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  emergency_write_crashdump( file, line);
  while(1)
    MPU_vTaskSuspend(0);
}

extern void finish_crash_handling(void);

void analyze_fault_stack(volatile unsigned int * hardfault_args)
{
  register_dump.stacked_r0 = ((unsigned long) hardfault_args[0]);
  register_dump.stacked_r1 = ((unsigned long) hardfault_args[1]);
  register_dump.stacked_r2 = ((unsigned long) hardfault_args[2]);
  register_dump.stacked_r3 = ((unsigned long) hardfault_args[3]);

  register_dump.stacked_r12 = ((unsigned long) hardfault_args[4]);
  register_dump.stacked_lr  = ((unsigned long) hardfault_args[5]);
  register_dump.stacked_pc  = ((unsigned long) hardfault_args[6]);
  register_dump.stacked_psr = ((unsigned long) hardfault_args[7]);

  register_dump.Hard_Fault_Status = *(uint32_t *) 0xe000ed2c;
  register_dump.IPSR = __get_IPSR();
//  register_dump.FPU_StatusControlRegister = __get_FPSCR(); has been saved before
  register_dump.Bad_Memory_Address = *(int*) 0xe000ed34;
  register_dump.Memory_Fault_status    = *(uint8_t*) 0xe000ed28;
  register_dump.Bus_Fault_Address=*(uint32_t *)0xe000ed38;
  register_dump.Bus_Fault_Status =*(uint8_t *) 0xe000ed29;
  register_dump.Usage_Fault_Status_Register = * (uint16_t *)0xe000ed2a;

  extern void * pxCurrentTCB;
  register_dump.active_TCB = pxCurrentTCB;

  finish_crash_handling();
}

void FPU_IRQHandler( void)
{
  register_dump.FPU_StatusControlRegister = __get_FPSCR();
  // patch FPFSR on the stack FPU context to avoid triggering the FPU IRQ recursively
  *(__IO uint32_t*)(FPU->FPCAR +0x40) = register_dump.FPU_StatusControlRegister & ~0x8f;
  __asm volatile
  (
      " mov r5, #0                                                     \n"
      " tst lr, #4                                                     \n"
      " ite eq                                                         \n"
      " mrseq r0, msp                                                  \n"
      " mrsne r0, psp                                                  \n"
      " ldr r1, [r0, #24]                                              \n"
      " ldr r2, handler1_address_const                                 \n"
      " bx r2                                                          \n"
      " handler1_address_const: .word analyze_fault_stack   	       \n"
  );
}

/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void
NMI_Handler(void)
{
  __asm volatile
  (
      " mov r5, #3                                                     \n"
      " tst lr, #4                                                     \n"
      " ite eq                                                         \n"
      " mrseq r0, msp                                                  \n"
      " mrsne r0, psp                                                  \n"
      " ldr r1, [r0, #24]                                              \n"
      " ldr r2, handler5u_address_const                                \n"
      " bx r2                                                          \n"
      " handler5u_address_const: .word analyze_fault_stack	       \n"
  );
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
      " mov r5, #0                                              \n"
      " tst lr, #4                                              \n"
      " ite eq                                                  \n"
      " mrseq r0, msp                                           \n"
      " mrsne r0, psp                                           \n"
      " ldr r1, [r0, #24]                                       \n"
      " ldr r2, handler2_address_const                          \n"
      " bx r2                                                   \n"
      " handler2_address_const: .word analyze_fault_stack	\n"
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
  register_dump.Bad_Memory_Address = *(int*) 0xe000ed34;
  register_dump.Memory_Fault_status    = *(uint8_t*) 0xe000ed28;
  // If you are stranded here:
  // Check Bad_Memory_Address and Bad_Instruction_Address !
	  // 0x92 : Stack overflow (push)
	  // 0x82 : Data access error
	  // 0x81 : Instruction access error

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
      " handler3_address_const: .word analyze_fault_stack   	       \n"
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
  register_dump.Bus_Fault_Address=*(uint32_t *)0xe000ed38;
  register_dump.Bus_Fault_Status =*(uint8_t *) 0xe000ed29;
	// bus fault status:
	// 0x92 stacking error address valid
	// 0x80 Bus_Fault_Address valid
	// 0x10 stacking error address INvalid
	// 0x08 un-stacking error
	// 0x04 imprecise error
	// 0x02 bus error during data access, size incorrect, disallowed in user mode ...
	// 0x01 branch to invalid memory, invalid EXC. return code, vector table error ...
  __asm volatile
  (
      " mov r5, #2                                                     \n"
      " tst lr, #4                                                     \n"
      " ite eq                                                         \n"
      " mrseq r0, msp                                                  \n"
      " mrsne r0, psp                                                  \n"
      " ldr r1, [r0, #24]                                              \n"
      " ldr r2, handler4_address_const                                 \n"
      " bx r2                                                          \n"
      " handler4_address_const: .word analyze_fault_stack  \n"
  );
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void __attribute__(( naked )) UsageFault_Handler(void)
{
  register_dump.Usage_Fault_Status_Register = * (uint16_t *)0xe000ed2a;
  __asm volatile
  (
      " mov r5, #3                                                     \n"
      " tst lr, #4                                                     \n"
      " ite eq                                                         \n"
      " mrseq r0, msp                                                  \n"
      " mrsne r0, psp                                                  \n"
      " ldr r1, [r0, #24]                                              \n"
      " ldr r2, handler3u_address_const                                \n"
      " bx r2                                                          \n"
      " handler3u_address_const: .word analyze_fault_stack	       \n"
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

/**
 * @brief  This function handles FreeRTOS's Stack Overflow exception.
 */
void
vApplicationStackOverflowHook(void)
{
  ASSERT(0);
}

/**
 * @brief  This function handles FreeRTOS's out of memory exception.
 */
void
vApplicationMallocFailedHook(void)
{
  ASSERT(0);
}

/**
 * @brief  This function handles FreeRTOS's out of memory exception.
 */
void illegal_interrupt_vector_hook(void)
{
  __asm volatile
  (
      " mov r5, #3                                                     \n"
      " tst lr, #4                                                     \n"
      " ite eq                                                         \n"
      " mrseq r0, msp                                                  \n"
      " mrsne r0, psp                                                  \n"
      " ldr r1, [r0, #24]                                              \n"
      " ldr r2, handler4u_address_const                                \n"
      " bx r2                                                          \n"
      " handler4u_address_const: .word analyze_fault_stack	       \n"
  );
}

void vTaskSuspend(uint32_t);

void vApplicationReturnFromTaskProcedureHook( void)
{
	ASSERT(0);
	vTaskSuspend(0);
}

void abort( void)
{
  ASSERT( 0);
}

#define SHORTCUT( fkt_name) \
void fkt_name(void) { while(1) { ASSERT(0);} }

//SHORTCUT( _sbrk)
SHORTCUT( _read)
//SHORTCUT( _write)
SHORTCUT( __cxa_pure_virtual)
SHORTCUT( __wrap___aeabi_unwind_cpp_pr0)
