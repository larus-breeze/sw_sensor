/***********************************************************************//**
 * @file		microphone_driver.h
 * @brief		ICS-43434 I2S microphone on I2S3: ENL and noise band levels
 * @license 		This project is released under the GNU Public License GPL-3.0
 **************************************************************************/
#ifndef MICROPHONE_DRIVER_H_
#define MICROPHONE_DRIVER_H_

#include "FreeRTOS_wrapper.h"
#include "microphone_analysis.h"

//! one record every 250 ms, logged as LRSX MICROPHONE_DATA
extern Queue <microphone_record_t> microphone_record_queue;

//! latest record, for the debugger's live view
extern microphone_record_t microphone_data;

//! consistent copy of the latest record, false without microphone or record
bool microphone_get_latest (microphone_record_t &record);

//! The same firmware runs with and without a microphone: it is detected at
//! start-up, see documentation/microphone_ENL.md
typedef enum
{
  MICROPHONE_NOT_BUILT = 0,	//!< firmware with RUN_MICROPHONE 0
  MICROPHONE_DETECTING = 1,	//!< first seconds after start-up, nothing logged yet
  MICROPHONE_PRESENT   = 2,	//!< records are logged, a later failure shows as ENL 000
  MICROPHONE_ABSENT    = 3	//!< no signal at start-up: I2S off, nothing logged
} microphone_state_t;

microphone_state_t microphone_get_state (void);

#endif /* MICROPHONE_DRIVER_H_ */
