#ifndef USD_HANDLER_H_
#define USD_HANDLER_H_

#include "FreeRTOS_wrapper.h"
#include "embedded_memory.h"
#include "reminder_flag.h"
#include "flexible_log_file_implementation.h"

extern bool logger_is_enabled;
extern bool magnetic_gound_calibration;
extern bool dump_sensor_readings;

enum SensorSdStatusFlags : uint8_t
{
  SENSOR_SD_PRESENT = 0x01,
  SENSOR_SD_MOUNTED = 0x02,
  SENSOR_SD_LOGGING_ENABLED = 0x04,
  SENSOR_SD_FLIGHT_LOG_ACTIVE = 0x08
};

extern uint8_t sensor_sd_status_flags;

extern flexible_log_file_implementation_t flex_file;
extern reminder_flag perform_after_landing_actions;
extern reminder_flag write_configuration_data_now;

#endif /* USD_HANDLER_H_ */
