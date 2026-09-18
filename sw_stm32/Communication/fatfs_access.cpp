/** *****************************************************************************
 * @file    	fatfs_access.cpp
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

#include "fatfs_access.h"
#include "common.h"

// static so only this module touches the raw Mutex - everyone else uses
// fatfs_lock()/fatfs_unlock()/fatfs_lock_best_effort().
COMMON static Mutex fatfs_mutex ((char*) "SD_ACCESS");

bool fatfs_lock (unsigned TicksToWait)
{
  return fatfs_mutex.lock (TicksToWait);
}

void fatfs_unlock (void)
{
  fatfs_mutex.unlock ();
}

// Can't wait normally (the crashed task itself might hold it) or skip
// locking (risks re-entering FatFs mid-mutation) - a short bounded
// attempt is the least-bad compromise.
#define FATFS_ACCESS_CRASH_TIMEOUT_MS  20u

bool fatfs_lock_best_effort (void)
{
  return fatfs_mutex.lock (pdMS_TO_TICKS (FATFS_ACCESS_CRASH_TIMEOUT_MS));
}
