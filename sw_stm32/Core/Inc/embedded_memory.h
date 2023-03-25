/** ***********************************************************************
 * @file	embedded_memory.h
 * @brief	COMMON data declarations
 * @author	Dr. Klaus Schaefer
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


#ifndef EMBEDDED_MEMORY_H_
#define EMBEDDED_MEMORY_H_

#define COMMON __attribute__ ((section ("common_data")))
#define CONSTEXPR_ROM constexpr __attribute__ ((section (".rodata")))
#ifndef ROM
#define ROM const __attribute__ ((section (".rodata")))
#endif
#endif /* EMBEDDED_MEMORY_H_ */
