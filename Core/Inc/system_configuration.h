/**
 * @file    system_configuration.h
 * @brief   system-wide tweaks
 * @author  Dr. Klaus Schaefer klaus.schaefer@h-da.de
 */
#ifndef SRC_SYSTEM_CONFIGURATION_H_
#define SRC_SYSTEM_CONFIGURATION_H_

#define RUN_MTi_1_MODULE 	0
#define RUN_MS5611_MODULE 	1
#define RUN_L3GD20			1
#define RUN_FXOS8700		0
#define RUN_PITOT_MODULE 	1
#define RUN_CAN_TESTER		0
#define ACTIVATE_USB		0

#define USE_DIFF_GNSS		0
#define UART3_LED_STATUS	0
#define UART4_LED_STATUS	0
#define uSD_LED_STATUS		1
#define RUN_GNSS_UPDATE_WITHOUT_FIX 0

#define RUN_OFFLINE_CALCULATION 0 // offline test mode
#define RUN_COMMUNICATOR	1 // normal mode
#define RUN_CAN_OUTPUT		0

#define RUN_DATA_LOGGER		1
#define LOG_OBSERVATIONS	0 // log IMU + pressure data
#define LOG_COORDINATES		0 // log GNSS data
#define LOG_OUTPUT_DATA		1 // logging all inclusive
#define OUTFILE ".f77"	// "LOG.F7x"
#define OLD_FORMAT 		0 // for year 2020 old data

#define RUN_SPI_TESTER		0
#define RUN_SDIO_TEST		0

#define MTI_PRIORITY		STANDARD_TASK_PRIORITY + 2

#define MS5611_PRIORITY		STANDARD_TASK_PRIORITY + 1
#define L3GD20_PRIORITY		STANDARD_TASK_PRIORITY + 1
#define PITOT_PRIORITY		STANDARD_TASK_PRIORITY + 1

#define COMMUNICATOR_PRIORITY	STANDARD_TASK_PRIORITY + 1

#define LOGGER_PRIORITY		STANDARD_TASK_PRIORITY
#define CAN_PRIORITY		STANDARD_TASK_PRIORITY
#define LOGGER_PRIORITY		STANDARD_TASK_PRIORITY

#endif /* SRC_SYSTEM_CONFIGURATION_H_ */
