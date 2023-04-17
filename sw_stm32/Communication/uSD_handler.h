#ifndef USD_HANDLER_H_
#define USD_HANDLER_H_

#include "FreeRTOS_wrapper.h"
#include "embedded_memory.h"

extern bool logger_is_enabled;
extern bool magnetic_gound_calibration;
extern bool dump_sensor_readings;

extern Semaphore magnetic_calibration_done;

#endif /* USD_HANDLER_H_ */
