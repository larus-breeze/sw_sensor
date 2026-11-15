// Host stub: lib's host configuration plus the firmware values uSD_handler.cpp uses.
#pragma once
#include "../../../lib/tests/stubs/system_configuration.h"
#define DISALLOW_DOWNGRADE 1
#define STANDARD_TASK_PRIORITY 1
#define LOGGER_PRIORITY (STANDARD_TASK_PRIORITY + 3)
#define U_SD_WRITE_TO_SYNC_FACTOR 50
