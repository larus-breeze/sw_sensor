/**
 * @file    	system_configuration.h
 * @brief   	system-wide tweaks
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

#ifndef SRC_SYSTEM_CONFIGURATION_H_
#define SRC_SYSTEM_CONFIGURATION_H_

#include "persistent_data.h"
#include "git-commit-version.h"

#define RUN_FLASH_WRITE_TESTER		0
#define ANALYZE_WRITE_PERFORMANCE	1

#define USE_HARDWARE_EEPROM		1
#define MEASURE_GNSS_REFRESH_TIME	0
#define ACTIVATE_USB_NMEA		1
#define CAN_RX_ERROR_REPORT		1
#define CRASFILE_ON_USER_RESET		1

#define RUN_GNSS			1
#define RUN_MTi_1_MODULE 		1
#define RUN_MS5611_MODULE 		1
#define RUN_PITOT_MODULE 		1

#define RUN_MICROPHONE			0

#define RUN_CAN_TESTER			0

#define ACTIVATE_USART_1_NMEA		1
#define ACTIVATE_USART_2_NMEA		1

#define ACTIVATE_BLUETOOTH_TEST		0
#define ACTIVATE_BLUETOOTH_HM19 	1

#define uSD_LED_STATUS			1

#define RUN_USART_1_TEST		0
#define RUN_USART_2_TEST		0

// task priorities

#define MTI_PRIORITY			STANDARD_TASK_PRIORITY + 6
#define MS5611_PRIORITY			STANDARD_TASK_PRIORITY + 6
#define PITOT_PRIORITY			STANDARD_TASK_PRIORITY + 6

#define COMMUNICATOR_PRIORITY		STANDARD_TASK_PRIORITY + 5

#define NMEA_USB_PRIORITY		STANDARD_TASK_PRIORITY + 3
#define NMEA_LISTEN_PRIORITY		STANDARD_TASK_PRIORITY + 3
#define BLUETOOTH_PRIORITY		STANDARD_TASK_PRIORITY + 3
#define CAN_PRIORITY			STANDARD_TASK_PRIORITY + 3

#define WATCHDOG_TASK_PRIORITY		STANDARD_TASK_PRIORITY + 2

#define LOGGER_PRIORITY			STANDARD_TASK_PRIORITY + 1

#define MAG_CALCULATOR_PRIORITY		STANDARD_TASK_PRIORITY
#define EEPROM_WRITER_PRIORITY	 	STANDARD_TASK_PRIORITY

// ISR priorities

#define EMERGENCY_ISR_PRIORITY		8 // highest priority
#define USB_ISR_PRIORITY		9
#define SDIO_ISR_PRIORITY		10

#define STANDARD_ISR_PRIORITY		14
#define WATCHDOG_ISR_PRIORITY		15 // lowest priority

// more parameters

#define FLASH_ISR_TIMEOUT		2
#define FLASH_ERASE_TIMEOUT		2000
#define FLASH_ACCESS_TIMEOUT		10
#define MAXIMUM_PAGE_ERASE_TIME 	2000

#define NMEA_REPORTING_PERIOD		250 // period in clock ticks for NMEA output
#define NMEA_DECIMATION_RATIO		0  // slow-down factor for the slow properties

#define RECURSIVE_LOCKS			1 // EEPROM mutex is recursive
#define MUTEX_TIMEOUT			2000 // timeout EEPROM mutex, enough to survive FLASH page erase

#define ACTIVATE_FPU_EXCEPTION_TRAP 	1 // I want to be SET !
#define SET_FPU_FLUSH_TO_ZERO		1
#define ACTIVATE_WATCHDOG		1
#define WATCHDOG_STATISTICS 		0
#define TRACE_ISR			0
#define INJECT_ERROR_NUMBER		0

void GNSS_data_lock( unsigned x);
#define ACQUIRE_GNSS_DATA_GUARD() GNSS_data_lock(1);
#define RELEASE_GNSS_DATA_GUARD() GNSS_data_lock(0);


#endif /* SRC_SYSTEM_CONFIGURATION_H_ */
