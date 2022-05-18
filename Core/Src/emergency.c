/** *****************************************************************************
 * @file    emergency.c
 * @author  Dr. Klaus Schaefer Hochschule Darmstadt schaefer.eit.h-da.de
 * @brief   Error handling routines
 ******************************************************************************/

#include <stdint.h>
#include "my_assert.h"
#include "stm32f407xx.h"

void emergency_write_crashdump( char * file, int line, uint64_t data);
void sync_logger(void );

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
uint32_t Memory_Fault_status;
uint32_t Bad_Instruction_Address;
uint32_t FPU_StatusControlRegister;
uint8_t  Bus_Fault_Status;
uint32_t Hard_Fault_Status;
uint16_t Usage_Fault_Status_Register;

/**
 * @brief  This function handles exceptions triggered by bad parameters to lib functions
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  emergency_write_crashdump( file, line, 0);
  MPU_vTaskSuspend(0);
  while(1)
    ;
}

void task_suspend_helper( void)
{
  sync_logger();
  while(1)
    MPU_vTaskSuspend(0);
}

void emergency_write_crashdump( char * file, int line, uint64_t data);

void analyze_fault_stack(volatile unsigned int * hardfault_args)
{
  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  hardfault_args[6] = task_suspend_helper;

  emergency_write_crashdump( __FILE__, __LINE__, (uint64_t)stacked_pc + ((uint64_t)stacked_lr << 32));
}

void FPU_IRQHandler( void)
{
  FPU_StatusControlRegister = __get_FPSCR();
  // patch FPFSR on the stack FPU context to avoid triggering the FPU IRQ recursively
  *(__IO uint32_t*)(FPU->FPCAR +0x40) = FPU_StatusControlRegister & ~0x8f;
  __asm volatile
  (
      " mov r5, #0                                                     \n"
      " tst lr, #4                                                     \n"
      " ite eq                                                         \n"
      " mrseq r0, msp                                                  \n"
      " mrsne r0, psp                                                  \n"
      " ldr r1, [r0, #24]                                              \n"
      " ldr r2, handler1_address_const                                \n"
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
  __asm volatile ( "bkpt 0" );
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void
HardFault_Handler(void)
{
  Hard_Fault_Status = *(uint32_t *) 0xe000ed2c;
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
  Bad_Memory_Address = *(int*) 0xe000ed34;
  Memory_Fault_status    = *(uint8_t*) 0xe000ed28;
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
      " ldr r2, handler3_address_const                                \n"
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
  Bus_Fault_Address=*(uint32_t *)0xe000ed38;
  Bus_Fault_Status =*(uint8_t *) 0xe000ed29;
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
      " ldr r2, handler4_address_const                                \n"
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
  Usage_Fault_Status_Register = * (uint16_t *)0xe000ed2a;
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
//SHORTCUT( _write)
SHORTCUT( __cxa_pure_virtual)
SHORTCUT( __wrap___aeabi_unwind_cpp_pr0)
