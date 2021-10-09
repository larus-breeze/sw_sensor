#ifndef COMMON_H
#define COMMON_H

extern uint32_t __common_data_start__[];
extern uint32_t __common_data_end__[];
#define COMMON_SIZE 8192 // cross-check linker *.ld file !
#define COMMON_BLOCK __common_data_start__
#define COMMON __attribute__ ((section ("common_data")))
#define ROM const __attribute__ ((section (".rodata")))

#ifdef __cplusplus

#include "GNSS.h"
extern GNSS_type GNSS;
#include "data_structures.h"
extern output_data_t output_data;
extern uint32_t UNIQUE_ID[4];

#endif // __cplusplus

#endif
