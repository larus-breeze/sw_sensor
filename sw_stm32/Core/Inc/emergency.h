/***********************************************************************//**
 * @file		emergency.h
 * @brief		definitions for crash handling
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 		This project is released under the GNU Public License GPL-3.0

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
  uint32_t IPSR;
  void * active_TCB;
} register_dump_t;

extern register_dump_t register_dump;

#endif /* INC_EMERGENCY_H_ */
