/** *****************************************************************************
 * @file    	fatfs_access.h
 * @brief   	centralized FatFs mutual-exclusion layer
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

#ifndef FATFS_ACCESS_H_
#define FATFS_ACCESS_H_

#include "FreeRTOS_wrapper.h"

//!< guards all FatFs access - not thread-safe (_FS_REENTRANT=0,
//!< ffconf.h). See documentation/wlan_link.md, "FatFs thread-safety".

//!< how long a WLAN-link request is willing to wait for FatFs access
//!< before giving up and replying WLAN_NACK_SD_CARD_BUSY
#define FATFS_ACCESS_WLAN_TIMEOUT_MS  500u

//!< Acquires exclusive FatFs access. TicksToWait in RTOS ticks (wrap ms in
//!< pdMS_TO_TICKS()); default waits forever. Pair with fatfs_unlock().
bool fatfs_lock (unsigned TicksToWait = INFINITE_WAIT);

//!< Releases what fatfs_lock()/fatfs_lock_best_effort() acquired.
void fatfs_unlock (void);

//!< For write_crash_dump() only. Bounded-time attempt; caller proceeds
//!< with its FatFs calls regardless of the result.
bool fatfs_lock_best_effort (void);

#endif /* FATFS_ACCESS_H_ */
