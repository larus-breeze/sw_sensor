/***********************************************************************//**
 * @file		inject_fault.cpp
 * @brief		Test routines for exception testing
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
    case 8:
	volatile unsigned d,e,f;
	d = 1; e = 1; f=0;
	d = e / f;
      break;
    case 9: // FPU underflow
	a=1.0;
	while( true)
	  {
	    a = a / 1000.0f;
	    delay(1);
	  }
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

