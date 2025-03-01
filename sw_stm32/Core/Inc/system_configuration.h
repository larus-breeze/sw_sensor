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

#define INCLUDING_NANO			1

#define WRITE_EEPROM_DEFAULTS		0
#define USE_HARDWARE_EEPROM		1
#define WITH_DENSITY_DATA		0
#define MEASURE_GNSS_REFRESH_TIME	0
#define HORIZON_DATA_SECRET		0
#define ACTIVATE_USB_NMEA		1
#define CAN_FORMAT_2021			1

#define ACTIVATE_MAGNETIC_3D_MECHANISM	0

#define WITH_LOWCOST_SENSORS	0 // 1 for Larus MK1 with L3GD20 gyro and FXOS8700 acc + mag chips
#define WITH_EXTERNAL_IMU	1

#define RUN_GNSS		1
#define RUN_MTi_1_MODULE 	1
#define RUN_MS5611_MODULE 	1
#define RUN_L3GD20 		0
#define RUN_FXOS8700		0
#define RUN_PITOT_MODULE 	1

#define RUN_MICROPHONE		0

#define RUN_CAN_TESTER		0
#define TEST_EEPROM		0

#define ACTIVATE_USART_1_NMEA	1
#define ACTIVATE_USART_2_NMEA	1

#define ACTIVATE_SENSOR_DUMP	0

#define ACTIVATE_BLUETOOTH_TEST	0
#define ACTIVATE_BLUETOOTH_HM19 1

#define uSD_LED_STATUS		1

#define RUN_SPI_TESTER		0
#define RUN_SDIO_TEST		0
#define RUN_USART_1_TEST	0
#define RUN_USART_2_TEST	0

// task priorities

#define MTI_PRIORITY		STANDARD_TASK_PRIORITY + 5

#define MS5611_PRIORITY		STANDARD_TASK_PRIORITY + 4
#define L3GD20_PRIORITY		STANDARD_TASK_PRIORITY + 4
#define PITOT_PRIORITY		STANDARD_TASK_PRIORITY + 4
#define COMMUNICATOR_PRIORITY	STANDARD_TASK_PRIORITY + 4
#define NMEA_USB_PRIORITY	STANDARD_TASK_PRIORITY + 4

#define BLUETOOTH_PRIORITY	STANDARD_TASK_PRIORITY + 3
#define CAN_PRIORITY		STANDARD_TASK_PRIORITY + 3

#define WATCHDOG_TASK_PRIORITY	STANDARD_TASK_PRIORITY + 2

#define LOGGER_PRIORITY		STANDARD_TASK_PRIORITY + 1

#define COMMUNICATOR_START_PRIORITY STANDARD_TASK_PRIORITY
#define MAG_CALCULATOR_PRIORITY	STANDARD_TASK_PRIORITY

// ISR priorities

#define EMERGENCY_ISR_PRIORITY	8 // highest priority
#define USB_ISR_PRIORITY	9
#define SDIO_ISR_PRIORITY	10

#define STANDARD_ISR_PRIORITY	14
#define WATCHDOG_ISR_PRIORITY	15 // lowest priority

// more parameters

#define NMEA_START_DELAY	10000

#define NMEA_REPORTING_PERIOD	250 // period in clock ticks for NMEA output
#define NMEA_DECIMATION_RATIO	6  // slow-down factor for the slow properties
#define FAST_NMEA_POSITION_OUTPUT 0

#define ACTIVATE_FPU_EXCEPTION_TRAP 1 // I want to be SET !
#define SET_FPU_FLUSH_TO_ZERO	1
#define ACTIVATE_WATCHDOG	1
#define WATCHDOG_STATISTICS 	0
#define TRACE_ISR		0
#define INJECT_ERROR_NUMBER	0

#endif /* SRC_SYSTEM_CONFIGURATION_H_ */
