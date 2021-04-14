/**
 * @file    system_configuration.h
 * @brief   system-wide tweaks
 * @author  Dr. Klaus Schaefer klaus.schaefer@h-da.de
 */
#ifndef SRC_SYSTEM_CONFIGURATION_H_
#define SRC_SYSTEM_CONFIGURATION_H_

#define DKCOM 1

#ifdef DKCOM

#define AVG_VARIO_F_BY_FS ( 0.5f / 30.0f / 10.0f) 	// assuming 10 Hz update
#define WIND_AVG_F_BY_FS ( 0.5f / 30.0f / 10.0f) 	// assuming 10 Hz update

#define WINDSPEED_F_BY_FS ( 1.0f / 5.0f / 100.0f) 	// 5s @ 100Hz
#define ACCELERATION_F_BY_FS ( 1.0f / 5.0f / 100.0f) 	// 5s @ 100Hz
#define VARIO_F_BY_FS ( 1.0f / 2.0f / 100.0f)      	// 2s @ 100Hz

#define USE_DIFF_GNSS		1

#define ALTI_DIFF 		0.136f 	// antenna height difference compensation
					// front lower for D-KCOM
#define HORIZ_DIFF		(-0.06f/2.03f)
#define DGNSS_SETUP_RAW		{ 2.03f, -0.06f, 0.136f} // slave antenna position
#define DGNSS_SETUP_NORMALIZED	{ 0.9973f, -0.0295f, 0.0668f} // slave antenna position norm.

#define BLUETOOTH_NAME		"AT+NAMED-KCOM"
#define ACTIVATE_USB_NMEA	1

#else // **************************************************************************

#define AVG_VARIO_F_BY_FS ( 0.5f / 30.0f / 10.0f) // assuming 10 Hz update
#define WIND_AVG_F_BY_FS ( 0.5f / 30.0f / 10.0f) // assuming 10 Hz update
#define WINDSPEED_F_BY_FS ( 1.0f / 5.0f / 100.0f) 	// 5s
#define ACCELERATION_F_BY_FS ( 1.0f / 5.0f / 100.0f) 	// 5s
#define VARIO_F_BY_FS ( 1.0 / 2.0f / 100.0f)      	// 2s

#define USE_DIFF_GNSS		0
#define ALTI_DIFF 		0.0 	// antenna height difference compensation
#define BLUETOOTH_NAME		"AT+NAMESOAR"
#define ACTIVATE_USB_NMEA	0
#endif

#define ACTIVATE_WATCHDOG	1

#define USE_GNSS_VARIO		1 	// else pressure-vario

#define RUN_OFFLINE_CALCULATION 0 // offline test mode, deprecated

#define INFILE "simin.f94"

#ifdef INFILE
#define RUN_GNSS		0
#define RUN_GNSS_HEADING	0
#define RUN_MTi_1_MODULE 	0
#define RUN_MS5611_MODULE 	0
#define RUN_L3GD20 		0
#define RUN_FXOS8700		0
#define RUN_PITOT_MODULE 	0
#else
#define RUN_GNSS		1
#define RUN_GNSS_HEADING	1
#define RUN_MTi_1_MODULE 	1
#define RUN_MS5611_MODULE 	1
#define RUN_L3GD20 		1
#define RUN_FXOS8700		1
#define RUN_PITOT_MODULE 	1
#endif

#define RUN_CAN_TESTER		0
#define TEST_EEPROM		0

#define ACTIVATE_BLUETOOTH_NMEA	1

#define ACTIVATE_USB_TEST	0
#define ACTIVATE_BLUETOOTH_TEST	0

#define UART3_LED_STATUS	0
#define UART4_LED_STATUS	0
#define D_GNSS_LED_STATUS	1
#define uSD_LED_STATUS		1
#define RUN_GNSS_UPDATE_WITHOUT_FIX 0

#define RUN_COMMUNICATOR	1 // normal mode
#define RUN_CAN_OUTPUT		1

#define RUN_DATA_LOGGER		1
#define LOG_OBSERVATIONS	0 // log IMU + pressure data
#define LOG_COORDINATES		0 // log GNSS data
#define LOG_OUTPUT_DATA		1 // logging all inclusive
#define OLD_FORMAT 		0 // for year 2020 old data without cheap sensor info
				  // and without GNSS speed information

#define RUN_SPI_TESTER		0
#define RUN_SDIO_TEST		0

#define MTI_PRIORITY		STANDARD_TASK_PRIORITY + 2

#define MS5611_PRIORITY		STANDARD_TASK_PRIORITY + 1
#define L3GD20_PRIORITY		STANDARD_TASK_PRIORITY + 1
#define PITOT_PRIORITY		STANDARD_TASK_PRIORITY + 1

#define COMMUNICATOR_PRIORITY	STANDARD_TASK_PRIORITY + 1

#define NMEA_USB_PRIORITY	STANDARD_TASK_PRIORITY + 2
#define BLUETOOTH_PRIORITY	STANDARD_TASK_PRIORITY
#define LOGGER_PRIORITY		STANDARD_TASK_PRIORITY
#define CAN_PRIORITY		STANDARD_TASK_PRIORITY
#define LOGGER_PRIORITY		STANDARD_TASK_PRIORITY

#define USB_ISR_PRIORITY	13
#define SDIO_ISR_PRIORITY	14
#define STANDARD_ISR_PRIORITY	15 // lowest priority

#define NMEA_REPORTING_PERIOD	250 // clock ticks

enum
{
	GNSS_AVAILABLE 		= 1,
	D_GNSS_AVAILABLE 	= 2,

	MTI_SENSOR_AVAILABE 	= 0x10,
	FXOS_SENSOR_AVAILABLE 	= 0x20,
	L3GD20_SENSOR_AVAILABLE = 0x40,
	MS5611_STATIC_AVAILABLE = 0x80,
	MS5611_PITOT_AVAILABLE  = 0x100,
	PITOT_SENSOR_AVAILABLE 	= 0x200,

	USB_OUTPUT_ACTIVE	= 0x1000,
	BLUEZ_OUTPUT_ACTIVE	= 0x2000,
	CAN_OUTPUT_ACTIVE	= 0x4000,
};

extern volatile unsigned system_state;

inline void update_system_state_set( unsigned value)
{
	__atomic_or_fetch ( &system_state, value, __ATOMIC_ACQUIRE);
}

#endif /* SRC_SYSTEM_CONFIGURATION_H_ */
