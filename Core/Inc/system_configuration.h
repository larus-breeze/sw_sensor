/**
 * @file    system_configuration.h
 * @brief   system-wide tweaks
 * @author  Dr. Klaus Schaefer klaus.schaefer@h-da.de
 */
#ifndef SRC_SYSTEM_CONFIGURATION_H_
#define SRC_SYSTEM_CONFIGURATION_H_

extern float * probe; // debugging probes

#include "persistent_data.h"

#define RUN_DATA_LOGGER		1

#define PARALLEL_MAGNETIC_AHRS	0 // run second AHRS without SAT compass usage
#define INCLUDING_NANO		1

#define WRITE_MAG_CALIB_EEPROM		1
#define LOG_MAGNETIC_CALIBRATION 	1
#define WRITE_EEPROM_DEFAULTS		0
#define USE_HARDWARE_EEPROM		1
#define WITH_DENSITY_DATA		0
#define GNSS_VERTICAL_SPEED_INVERTED	0 // for simulation with old data
#define MEASURE_GNSS_REFRESH_TIME	0
#define USE_LARUS_NMEA_EXTENSIONS	1

#define DKCOM 				0

#define AVG_VARIO_F_BY_FS 	( 1.0f / 30.0f / 10.0f) 	// assuming 10 Hz update
#define WIND_AVG_F_BY_FS 	( 1.0f / 30.0f / 10.0f) 	// assuming 10 Hz update

#define WIND_SHORTTERM_F_BY_FS 	( 1.0f / 5.0f / 100.0f) 	// 5s @ 100Hz
#define VARIO_F_BY_FS          	( 1.0f / 2.0f / 100.0f)      	// 2s @ 100Hz

#define ACTIVATE_USB_NMEA	0

#define WITH_LOWCOST_SENSORS	0

#define RUN_GNSS		1
#define RUN_MTi_1_MODULE 	1
#define RUN_MS5611_MODULE 	1
#define RUN_L3GD20 		0
#define RUN_FXOS8700		0
#define RUN_PITOT_MODULE 	1

#define RUN_CAN_TESTER		0
#define TEST_EEPROM		0

#define ACTIVATE_USART_1_NMEA	1
#define ACTIVATE_USART_2_NMEA	1

#define ACTIVATE_USB_TEST	0
#define ACTIVATE_BLUETOOTH_TEST	0

#define uSD_LED_STATUS		1

#define RUN_SPI_TESTER		0
#define RUN_SDIO_TEST		0
#define RUN_USART_1_TEST	0
#define RUN_USART_2_TEST	0

#define MTI_PRIORITY		STANDARD_TASK_PRIORITY + 3

#define MS5611_PRIORITY		STANDARD_TASK_PRIORITY + 2
#define L3GD20_PRIORITY		STANDARD_TASK_PRIORITY + 2
#define PITOT_PRIORITY		STANDARD_TASK_PRIORITY + 2

#define COMMUNICATOR_PRIORITY	STANDARD_TASK_PRIORITY + 2

#define NMEA_USB_PRIORITY	STANDARD_TASK_PRIORITY + 3
#define BLUETOOTH_PRIORITY	STANDARD_TASK_PRIORITY + 1
#define LOGGER_PRIORITY		STANDARD_TASK_PRIORITY
#define CAN_PRIORITY		STANDARD_TASK_PRIORITY + 1
#define WATCHDOG_TASK_PRIORITY	STANDARD_TASK_PRIORITY + 1 // todo change me to be lowest prio some day

#define EMERGENCY_ISR_PRIORITY	12 // highest priority
#define USB_ISR_PRIORITY	13
#define SDIO_ISR_PRIORITY	14
#define STANDARD_ISR_PRIORITY	15 // lowest priority

#define NMEA_REPORTING_PERIOD	250 // period in clock ticks for NMEA output

#define ACTIVATE_FPU_EXCEPTION_TRAP 0 // todo I want to be SET !
#define SET_FPU_FLUSH_TO_ZERO	1
#define ACTIVATE_WATCHDOG	1
#define WATCHDOG_STATISTICS 	0

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
	AIR_SENSOR_AVAILABLE 	= 0x400,

	USB_OUTPUT_ACTIVE	= 0x1000,
	USART_1_OUTPUT_ACTIVE	= 0x2000,
	CAN_OUTPUT_ACTIVE	= 0x4000,
	USART_2_OUTPUT_ACTIVE	= 0x8000
};

extern volatile unsigned system_state;

inline void update_system_state_set( unsigned value)
{
	__atomic_or_fetch ( &system_state, value, __ATOMIC_ACQUIRE);
}

inline void update_system_state_clear( unsigned value)
{
	__atomic_and_fetch ( &system_state, ~value, __ATOMIC_ACQUIRE);
}

#endif /* SRC_SYSTEM_CONFIGURATION_H_ */
