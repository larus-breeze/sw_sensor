/**
 * @file    system_configuration.h
 * @brief   system-wide tweaks
 * @author  Dr. Klaus Schaefer klaus.schaefer@h-da.de
 */
#ifndef SRC_SYSTEM_CONFIGURATION_H_
#define SRC_SYSTEM_CONFIGURATION_H_

#include "persistent_data.h"

//#define INFILE "simin_20210530.f46" // switches on offline calculation and defines filename
//#define IN_DATA_LENGTH 46
//#define OUTFILE "simout_0707_20210530.f97"
//#define OUTFILE "data.f24"
//#define MAXSPEED_CALCULATION	1 // not realtime but 100% CPU duty cycle

#define LOG_MAGNETIC_CALIBRATION 	0
#define WRITE_EEPROM_DEFAULTS		0

#define DKCOM 1

#define AVG_VARIO_F_BY_FS 	( 1.0f / 30.0f / 10.0f) 	// assuming 10 Hz update
#define WIND_AVG_F_BY_FS 	( 1.0f / 30.0f / 10.0f) 	// assuming 10 Hz update

#define WIND_SHORTTERM_F_BY_FS 	( 1.0f / 5.0f / 100.0f) 	// 5s @ 100Hz
#define VARIO_F_BY_FS          	( 1.0f / 2.0f / 100.0f)      	// 2s @ 100Hz

#define DISABLE_CIRCLING_STATE 	0
#if DISABLE_CIRCLING_STATE != 1
#define CIRCLE_LIMIT 		(10 * 100) //!< 10 * 1/100 s delay into / out of circling state
#define STABLE_CIRCLING_LIMIT	(30 * 100) // seconds @ 100 Hz for MAG auto calibration
#endif

#define MINIMUM_MAG_CALIBRATION_SAMPLES 6000
#define MAG_CALIB_LETHARGY	0.8f // percentage of remaining old calibration info
#define MAG_CALIBRATION_CHANGE_LIMIT 5.0e-4f // variance average of changes: 3 * { offset, scale }

#define GNSS_SAMPLE_RATE 	10.0f // depending on master GNSS RX configuration

#define USE_GNSS_VARIO		1 // else pressure-vario

#if DKCOM == 1 // *******************************************************************


#define BLUETOOTH_NAME		"AT+NAMED-KCOM"
#define ACTIVATE_USB_NMEA	1

#else // **************************************************************************

#define BLUETOOTH_NAME		"AT+NAMEALBATROS2"
#define ACTIVATE_USB_NMEA	0

#endif // **************************************************************************

#define ACTIVATE_WATCHDOG	1

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
#define RUN_MTi_1_MODULE 	1
#define RUN_MS5611_MODULE 	1
#define RUN_L3GD20 		0
#define RUN_FXOS8700		0
#define RUN_PITOT_MODULE 	1

#endif

#define RUN_CAN_TESTER		0
#define TEST_EEPROM		0

#define ACTIVATE_BLUETOOTH_NMEA	1
#define ACTIVATE_USART_2_NMEA	1

#define ACTIVATE_USB_TEST	0
#define ACTIVATE_BLUETOOTH_TEST	0

#define uSD_LED_STATUS		1

#define RUN_COMMUNICATOR	1 // normal mode
#define RUN_CAN_OUTPUT		1

#define RUN_DATA_LOGGER		1
#define LOG_OBSERVATIONS	1 // log IMU + pressure data
#define LOG_COORDINATES		1 // log GNSS data
#define LOG_OUTPUT_DATA		0 // logging all inclusive
#define OLD_FORMAT 		0 // for year 2020 old data without cheap sensor info
				  // and without GNSS speed information

#define RUN_SPI_TESTER		0
#define RUN_SDIO_TEST		0
#define RUN_USART_2_TEST	0

#define MTI_PRIORITY		(STANDARD_TASK_PRIORITY + 2)

#define MS5611_PRIORITY		(STANDARD_TASK_PRIORITY + 1)
#define L3GD20_PRIORITY		(STANDARD_TASK_PRIORITY + 1)
#define PITOT_PRIORITY		(STANDARD_TASK_PRIORITY + 1)

#define COMMUNICATOR_PRIORITY	(STANDARD_TASK_PRIORITY + 1)

#define NMEA_USB_PRIORITY	(STANDARD_TASK_PRIORITY + 2)
#define BLUETOOTH_PRIORITY	STANDARD_TASK_PRIORITY
#define LOGGER_PRIORITY		STANDARD_TASK_PRIORITY
#define CAN_PRIORITY		STANDARD_TASK_PRIORITY
#define LOGGER_PRIORITY		STANDARD_TASK_PRIORITY

#define EMERGENCY_ISR_PRIORITY	12 // highest priority
#define USB_ISR_PRIORITY	13
#define SDIO_ISR_PRIORITY	14
#define STANDARD_ISR_PRIORITY	15 // lowest priority

#define NMEA_REPORTING_PERIOD	250 // period in clock ticks for NMEA output

#define ACTIVATE_FPU_EXCEPTION_TRAP 	0 // todo I want to be SET !
#define SET_FPU_FLUSH_TO_ZERO		1

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
	USART_2_OUTPUT_ACTIVE	= 0x8000
};

extern volatile unsigned system_state;

inline void update_system_state_set( unsigned value)
{
	__atomic_or_fetch ( &system_state, value, __ATOMIC_ACQUIRE);
}

#endif /* SRC_SYSTEM_CONFIGURATION_H_ */
