#ifndef COMMON_H
#define COMMON_H

extern uint32_t __common_data_start__[];
extern uint32_t __common_data_end__[];
#define COMMON_SIZE 8192 // cross-check linker *.ld file !
#define COMMON_BLOCK __common_data_start__
#define COMMON __attribute__ ((section ("common_data")))
#define ROM const __attribute__ ((section (".rodata")))

#endif
