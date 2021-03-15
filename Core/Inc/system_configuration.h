/**
 * @file    system_configuration.h
 * @brief   system-wide tweaks
 * @author  Dr. Klaus Schaefer klaus.schaefer@h-da.de
 */
#ifndef SRC_SYSTEM_CONFIGURATION_H_
#define SRC_SYSTEM_CONFIGURATION_H_

#define RUN_MTi_1_MODULE 	0
#define RUN_MS5611_MODULE 	0
#define RUN_L3GD20 		1
#define RUN_FXOS8700		1
#define RUN_PITOT_MODULE 	1
#define RUN_CAN_TESTER		0

#define ACTIVATE_USB_NMEA	0
#define ACTIVATE_BLUETOOTH_NMEA	0

#define ACTIVATE_USB_TEST	0
#define ACTIVATE_BLUETOOTH_TEST	0

#define USE_DIFF_GNSS		1
#define UART3_LED_STATUS	0
#define UART4_LED_STATUS	0
#define uSD_LED_STATUS		1
#define RUN_GNSS_UPDATE_WITHOUT_FIX 0

#define RUN_OFFLINE_CALCULATION 0 // offline test mode
#define RUN_COMMUNICATOR	1 // normal mode
#define RUN_CAN_OUTPUT		1

#define RUN_DATA_LOGGER		1
#define LOG_OBSERVATIONS	1 // log IMU + pressure data
#define LOG_COORDINATES		1 // log GNSS data
#define LOG_OUTPUT_DATA		0 // logging all inclusive
#define OLD_FORMAT 		0 // for year 2020 old data

#define RUN_SPI_TESTER		0
#define RUN_SDIO_TEST		0

#define MTI_PRIORITY		STANDARD_TASK_PRIORITY + 2

#define MS5611_PRIORITY		STANDARD_TASK_PRIORITY + 1
#define L3GD20_PRIORITY		STANDARD_TASK_PRIORITY + 1
#define PITOT_PRIORITY		STANDARD_TASK_PRIORITY + 1

#define COMMUNICATOR_PRIORITY	STANDARD_TASK_PRIORITY + 1

#define BLUETOOTH_PRIORITY	STANDARD_TASK_PRIORITY
#define LOGGER_PRIORITY		STANDARD_TASK_PRIORITY
#define CAN_PRIORITY		STANDARD_TASK_PRIORITY
#define LOGGER_PRIORITY		STANDARD_TASK_PRIORITY

#define NMEA_REPORTING_PERIOD	250 // clock ticks

enum
{
	GNSS_AVAILABLE 			= 1,
	D_GNSS_AVAILABLE 		= 2,

	MTI_SENSOR_AVAILABE 	= 0x10,
	FXOS_SENSOR_AVAILABLE 	= 0x20,
	L3GD20_SENSOR_AVAILABLE = 0x40,
	MS5611_STATIC_AVAILABLE = 0x80,
	MS5611_PITOT_AVAILABLE  = 0x100,
	PITOT_SENSOR_AVAILABLE 	= 0x200,

	USB_OUTPUT_ACTIVE		= 0x1000,
	BLUEZ_OUTPUT_ACTIVE		= 0x2000,
	CAN_OUTPUT_ACTIVE		= 0x4000,
};

extern volatile unsigned system_state;

inline void update_system_state_set( unsigned value)
{
	__atomic_or_fetch ( &system_state, value, __ATOMIC_ACQUIRE);
}

#endif /* SRC_SYSTEM_CONFIGURATION_H_ */
