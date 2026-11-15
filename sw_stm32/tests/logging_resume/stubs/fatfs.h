// Host stub: the FatFs calls uSD_handler.cpp makes; behaviour in logging_resume_test.cpp.
#pragma once
#include <stdint.h>
typedef uint32_t DWORD;
typedef unsigned UINT;
typedef enum { FR_OK = 0, FR_DISK_ERR, FR_NO_FILE = 4 } FRESULT;
typedef struct { uint16_t csize; uint16_t ssize; } FATFS;
typedef struct { int dummy; } FIL;
typedef struct { uint8_t fattrib; } FILINFO;
#define AM_DIR 0x10
#define FA_READ 0x01
FRESULT f_mount (FATFS *, const char *, uint8_t);
FRESULT f_open (FIL *, const char *, uint8_t);
FRESULT f_close (FIL *);
FRESULT f_stat (const char *, FILINFO *);
FRESULT f_rename (const char *, const char *);
FRESULT f_getfree (const char *, DWORD *, FATFS **);
