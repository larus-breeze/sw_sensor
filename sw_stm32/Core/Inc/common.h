/***********************************************************************//**
 * @file		common.h
 * @brief		Definitions for COMMON memory
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
#ifndef COMMON_H
#define COMMON_H

extern uint32_t __common_data_start__[];
extern uint32_t __common_data_end__[];
#define COMMON_SIZE 16384 // cross-check linker *.ld file !
#define COMMON_BLOCK __common_data_start__
#define COMMON __attribute__ ((section ("common_data")))

#endif
