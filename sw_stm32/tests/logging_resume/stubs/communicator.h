// Host stub: what uSD_handler.cpp needs from the communicator.
#pragma once
#include "data_structures.h"
extern D_GNSS_coordinates_t coordinates;
extern bool host_airborne; // set by the test scenario
static inline bool is_airborne (void) { return host_airborne; }
