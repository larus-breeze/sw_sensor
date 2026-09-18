#ifndef USD_HANDLER_H_
#define USD_HANDLER_H_

#include "FreeRTOS_wrapper.h"
#include "embedded_memory.h"
#include "reminder_flag.h"
#include "flexible_log_file_implementation.h"
#include "fatfs_access.h"

extern bool logger_is_enabled;
extern bool magnetic_gound_calibration;
extern bool dump_sensor_readings;
extern bool sd_card_mounted;
extern bool logging_active;
extern bool logging_paused_by_user;
extern bool logging_force_start;

// One-shot "flash update already attempted this power-cycle" marker - see
// firmware_update_retry_marker's doc comment, uSD_handler.cpp, for why it
// survives a warm reset but not a power cycle. Set by
// read_software_update() (uSD_helpers.cpp) right after staging an image
// into flash; consulted by jump_to_pending_flash_update_if_any()
// (uSD_handler.cpp) to allow at most one jump attempt per power-cycle.
#define FIRMWARE_UPDATE_JUST_STAGED    0x12344321u
#define FIRMWARE_UPDATE_ALREADY_TRIED  0x98766789u
extern uint32_t firmware_update_retry_marker;

// Diagnostic counters, reset to 0 only on a genuine power-on/brown-out
// reset (checked once, at the top of uSD_handler_runnable()) - a warm
// reset (WWDG, SCB::sys_reset(), NRST pin) leaves them counting across the
// reset instead of restarting from 0, same NoInit mechanism as
// firmware_update_retry_marker above.
extern uint32_t reset_count;       //!< number of resets since the last power-on
extern uint32_t flash_erase_count; //!< number of flash sector erases issued since the last power-on

extern flexible_log_file_implementation_t flex_file;
extern reminder_flag perform_after_landing_actions;
extern reminder_flag write_configuration_data_now;

// FatFs access goes through fatfs_lock()/fatfs_unlock() (fatfs_access.h).
// uSD_handler_task's own lock call waits indefinitely - a takeoff during a
// WLAN upload/format delays the start of logging rather than aborting the
// in-progress operation (deliberate: see documentation/wlan_link.md,
// "FatFs thread-safety").

#endif /* USD_HANDLER_H_ */
