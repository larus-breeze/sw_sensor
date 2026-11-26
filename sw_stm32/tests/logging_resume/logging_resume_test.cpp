// Host test for the logging pause in uSD_handler.cpp (issue #302).
//
// Builds the real uSD_handler_runnable() with FatFs, HAL and FreeRTOS
// simulated: time advances by 100 ms in delay() and by 10 ms per
// notify_take() (the communicator's 100 Hz sync). Each scenario plays the
// web UI's stop/start requests (wlan_link_handler.cpp) or a takeoff and
// checks when the handler opens and closes log files. The endless task is
// left by throwing from the simulated time.
//
//   resume_after_stop   "Stop logging", then "Resume logging" while on
//                       ground: a new file must be opened (#302).
//   resume_at_takeoff   a forgotten pause must be cleared at takeoff, so
//                       the flight is logged (d9f1e88).
//   retry_when_sd_full  a full card blocks logging; once there is room
//                       again, logging starts (re-check every ~10 s).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <functional>
#include <sys/mman.h>

#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "fatfs.h"
#include "communicator.h"
#include "uSD_handler.h"
#include "uSD_helpers.h"
#include "emergency.h"

void uSD_handler_runnable (void *);

// ---- simulated hardware and environment ------------------------------------

GPIO_TypeDef host_gpio;
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx, hdma_sdio_tx;
uint64_t FAT_time;
D_GNSS_coordinates_t coordinates;
bool host_airborne;
Semaphore setup_file_handling_completed;
Semaphore watchdog_activator;
RestrictedTask watchdog_handler (0, "WDG", 0, 0, 0);
flexible_log_file_implementation_t flex_file;
char *crashfile;
unsigned crashline;
void *pxCurrentTCB;
register_dump_t register_dump;
uint8_t firmware_SHA256_digest[32];

static bool sd_card_full;

HAL_StatusTypeDef HAL_SD_Init (SD_HandleTypeDef *) { return HAL_OK; }
HAL_StatusTypeDef HAL_SD_DeInit (SD_HandleTypeDef *) { return HAL_OK; }
bool BSP_PlatformIsDetected (void) { return true; }

FRESULT f_mount (FATFS *, const char *, uint8_t) { return FR_OK; }
FRESULT f_open (FIL *, const char *, uint8_t) { return FR_NO_FILE; } // no "sensor.readings"
FRESULT f_close (FIL *) { return FR_OK; }
FRESULT f_rename (const char *, const char *) { return FR_OK; }
FRESULT f_stat (const char *path, FILINFO *info)
{
  if (strcmp (path, "logger") == 0)
    {
      info->fattrib = AM_DIR;
      return FR_OK;
    }
  return FR_NO_FILE; // no "eeprom" entry: the handler writes its EEPROM dump
}
FRESULT f_getfree (const char *, DWORD *free_clusters, FATFS **fs)
{
  static FATFS volume = { 64, 512 }; // 32 KiB clusters
  *free_clusters = sd_card_full ? 10 : 100000;
  *fs = &volume;
  return FR_OK;
}

bool fatfs_lock (unsigned) { return true; }
void fatfs_unlock (void) {}
bool read_init_file (const char *) { return false; }
void recover_and_initialize_flash (void) {}
bool ensure_EEPROM_parameter_integrity (void) { return true; }
uint32_t wlan_link_crc32_hw_compute (const uint8_t *, uint32_t) { return 0; }
bool read_software_update (void) { return false; }
void write_crash_dump (bool) { abort (); }
bool crash_report_pending (void) { return false; }
void write_pending_crash_report (void) {}
void write_reboot_report (const char *, FIL &) {}
bool write_EEPROM_dump (const char *) { return true; }
void make_firmware_digest (void) {}
static unsigned file_number; // stands in for the date and time in the file name
char * format_date_time (char *target, const D_GNSS_coordinates_t &)
{
  return target + sprintf (target, "file%02u", ++file_number);
}

// uSD_handler.cpp reads the flash staging area at 0x08060000-0x080bffff
// directly. Map it as erased flash (0xff), so there is no update to jump to
// and nothing to erase.
__attribute__((constructor)) static void map_flash_staging_area (void)
{
  void *want = (void *) 0x08060000;
  void *got = mmap (want, 0x60000, PROT_READ | PROT_WRITE,
		    MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
  if (got != want)
    {
      fprintf (stderr, "cannot map the simulated flash at 0x08060000\n");
      exit (2);
    }
  memset (got, 0xff, 0x60000);
}

// ---- log files ------------------------------------------------------------

struct log_file { std::string name; unsigned opened_ms; unsigned closed_ms; };
static std::vector<log_file> files;
static unsigned now_ms;

bool flexible_log_file_implementation_t::open (const char *file_name)
{
  files.push_back ({ file_name, now_ms, 0 });
  return true;
}
void flexible_log_file_implementation_t::close (void)
{
  if (! files.empty () && files.back ().closed_ms == 0)
    files.back ().closed_ms = now_ms;
}

// ---- simulated time and scenarios -------------------------------------------

struct scenario_end { bool passed; std::string reason; };
static std::function<void (void)> scenario_step;

void delay (unsigned ms) { now_ms += ms; scenario_step (); }
uint32_t notify_take (bool) { now_ms += 10; scenario_step (); return 1; }

static void fail (const std::string &reason) { throw scenario_end { false, reason }; }
static void pass (const std::string &reason) { throw scenario_end { true, reason }; }

static std::string at (unsigned ms)
{
  char buffer[32];
  snprintf (buffer, sizeof buffer, "t=%.1f s", ms / 1000.0);
  return buffer;
}

static std::string after (unsigned ms)
{
  char buffer[32];
  snprintf (buffer, sizeof buffer, "%u ms", ms);
  return buffer;
}

// Runs one scenario: its step() is called after every simulated delay.
static bool run (const char *name, std::function<void (void)> step)
{
  files.clear ();
  file_number = 0;
  now_ms = 0;
  host_airborne = false;
  sd_card_full = false;
  logging_active = logging_paused_by_user = logging_force_start = false;
  logging_stopped_sd_full = false;
  memset ((void *) &coordinates, 0, sizeof coordinates);
  coordinates.sat_fix_type = 1; // GNSS time is available
  scenario_step = step;
  scenario_end end { false, "" };
  try
    {
      uSD_handler_runnable (0);
    }
  catch (scenario_end &e)
    {
      end = e;
    }
  printf ("%s %-20s %s\n", end.passed ? "PASS" : "FAIL", name, end.reason.c_str ());
  for (const log_file &f : files)
    printf ("     %-12s opened %s, %s\n", f.name.c_str (), at (f.opened_ms).c_str (),
	    f.closed_ms ? ("closed " + at (f.closed_ms)).c_str () : "still open");
  return end.passed;
}

// The common start: the handler opens the first file shortly after boot.
// Then stop logging like the web UI's "Stop logging" (wlan_link_handler.cpp)
// and wait until the file is closed. Returns false while still in progress.
struct stop_sequence
{
  unsigned stop_ms = 0;
  bool done = false;

  bool step (void)
  {
    if (done)
      return true;
    if (files.empty ())
      {
	if (now_ms > 5000)
	  fail ("no log file opened within 5 s after boot");
	return false;
      }
    if (stop_ms == 0 && now_ms >= files[0].opened_ms + 3000)
      {
	logging_paused_by_user = true; // handle_stop_logging_request()
	stop_ms = now_ms;
      }
    if (stop_ms != 0)
      {
	if (files[0].closed_ms != 0 && ! logging_active)
	  done = true;
	else if (now_ms > stop_ms + 1000)
	  fail ("file not closed within 1 s after \"Stop logging\"");
      }
    return done;
  }
};

static bool resume_after_stop (void)
{
  stop_sequence stop;
  unsigned resume_ms = 0;
  return run ("resume_after_stop", [&] {
    if (! stop.step ())
      return;
    if (resume_ms == 0 && now_ms >= stop.stop_ms + 10000)
      {
	logging_paused_by_user = false; // handle_start_logging_request()
	logging_force_start = true;
	resume_ms = now_ms;
      }
    if (resume_ms == 0)
      return;
    if (files.size () >= 2 && logging_active)
      pass ("new file " + after (files[1].opened_ms - resume_ms) + " after \"Resume logging\"");
    if (now_ms > resume_ms + 5000)
      fail ("no new file within 5 s after \"Resume logging\" (logging_active="
	    + std::string (logging_active ? "true" : "false") + ")");
  });
}

static bool resume_at_takeoff (void)
{
  stop_sequence stop;
  unsigned takeoff_ms = 0;
  return run ("resume_at_takeoff", [&] {
    if (! stop.step ())
      return;
    if (takeoff_ms == 0 && now_ms >= stop.stop_ms + 10000)
      {
	host_airborne = true; // pause forgotten, the glider takes off
	takeoff_ms = now_ms;
      }
    if (takeoff_ms == 0)
      return;
    if (files.size () >= 2 && logging_active)
      pass ("new file " + after (files[1].opened_ms - takeoff_ms) + " after takeoff");
    if (now_ms > takeoff_ms + 5000)
      fail ("no new file within 5 s after takeoff (logging_paused_by_user="
	    + std::string (logging_paused_by_user ? "true" : "false") + ")");
  });
}

static bool retry_when_sd_full (void)
{
  unsigned room_ms = 0;
  sd_card_full = true;
  return run ("retry_when_sd_full", [&] {
    if (now_ms < 2000)
      sd_card_full = true; // full card at boot
    if (! files.empty () && room_ms == 0)
      fail ("log file opened although the card is full");
    if (room_ms == 0 && now_ms >= 15000)
      {
	if (! logging_stopped_sd_full)
	  fail ("logging_stopped_sd_full not set on a full card");
	sd_card_full = false; // files deleted via the web UI
	room_ms = now_ms;
      }
    if (room_ms == 0)
      return;
    if (! files.empty () && logging_active)
      pass ("logging started " + after (files[0].opened_ms - room_ms) + " after space was freed");
    if (now_ms > room_ms + 15000)
      fail ("logging did not start within 15 s after space was freed");
  });
}

int main (int argc, char **argv)
{
  const char *which = argc > 1 ? argv[1] : "all";
  bool all = strcmp (which, "all") == 0;
  int failures = 0;
  if (all || strcmp (which, "resume_after_stop") == 0)
    failures += ! resume_after_stop ();
  if (all || strcmp (which, "resume_at_takeoff") == 0)
    failures += ! resume_at_takeoff ();
  if (all || strcmp (which, "retry_when_sd_full") == 0)
    failures += ! retry_when_sd_full ();
  return failures ? 1 : 0;
}
