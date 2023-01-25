/*
 * emergency.h
 *
 *  Created on: Jan 25, 2023
 *      Author: schaefer
 */

#ifndef INC_EMERGENCY_H_
#define INC_EMERGENCY_H_

#ifdef __cplusplus
 extern "C"
#endif
void emergency_write_crashdump( char * file, int line);

typedef struct
{
  uint32_t  stacked_r0;
  uint32_t  stacked_r1;
  uint32_t  stacked_r2;
  uint32_t  stacked_r3;
  uint32_t  stacked_r12;
  uint32_t  stacked_lr;
  uint32_t  stacked_pc;
  uint32_t  stacked_psr;

  uint32_t Bus_Fault_Address;
  uint32_t Bad_Memory_Address;
  uint32_t Memory_Fault_status;
  uint32_t FPU_StatusControlRegister;
  uint32_t Bus_Fault_Status;
  uint32_t Hard_Fault_Status;
  uint32_t Usage_Fault_Status_Register;

  void * active_TCB;
} register_dump_t;

extern register_dump_t register_dump;

#endif /* INC_EMERGENCY_H_ */
