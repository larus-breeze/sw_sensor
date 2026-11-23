/** *****************************************************************************
 * @file    	uSD_helpers.h
 * @brief   	uSD File r/w helper functions
 * @author  	Dr. Klaus Schaefer
 * @copyright 	Copyright 2026 Dr. Klaus Schaefer. All rights reserved.
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

#ifndef USD_HELPERS_H_
#define USD_HELPERS_H_

extern char *crashfile;
extern unsigned crashline;

bool read_software_update (void);
//!< user_initiated: an intentional reset (see reset_by_watchdog()), written
//!< as *.REBOOT instead of *.CRASHDUMP
void write_crash_dump( bool user_initiated);
bool crash_report_pending (void);
void write_pending_crash_report (void);
bool write_EEPROM_dump( const char * file_path);
char * format_date_time( char * target, const D_GNSS_coordinates_t &coordinates);
void make_firmware_digest( void);
extern uint8_t firmware_SHA256_digest[32];

#endif /* USD_HELPERS_H_ */
