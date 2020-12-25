/**
 * @file    system_configuration.h
 * @brief   system-wide tweaks
 * @author  Dr. Klaus Schaefer klaus.schaefer@h-da.de
 */
#ifndef SRC_SYSTEM_CONFIGURATION_H_
#define SRC_SYSTEM_CONFIGURATION_H_

#define RUN_MTi_1_MODULE 	1
#define RUN_MS5611_MODULE 	1
#define RUN_CHIPSENS_MODULE 	1
#define RUN_PITOT_MODULE 	1
#define RUN_CAN_TESTER		1
#define ACTIVATE_USB		1

#define USE_DIFF_GNSS		1
#define UART3_LED_STATUS	0
#define UART4_LED_STATUS	0
#define uSD_LED_STATUS		1

#define RUN_DATA_LOGGER		1
#define LOG_COORDINATES		1

#define RUN_SPI_TESTER		0
#define RUN_SDIO_TEST		0

#endif /* SRC_SYSTEM_CONFIGURATION_H_ */
