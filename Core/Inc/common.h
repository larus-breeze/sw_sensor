#ifndef COMMON_H
#define COMMON_H

#include "stdint.h"

extern uint32_t __common_data_start__[];
extern uint32_t __common_data_end__[];
#define COMMON_SIZE 16384
#define COMMON_BLOCK __common_data_start__
#define COMMON __attribute__ ((section ("common_data")))
#define ROM const __attribute__ ((section (".rodata")))

#endif
