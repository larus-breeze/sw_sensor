/**
 * @file    my_assert.h
 * @brief   special ASSERT macro
 * @author  Dr. Klaus Schaefer klaus.schaefer@h-da.de
 */

#ifndef MY_ASSERT_H_
#define MY_ASSERT_H_

#include "stdint.h"

#ifdef __cplusplus
 extern "C"
#endif
   void emergency_write_crashdump( char * file, int line, uint64_t data);

#if 0
#define ASSERT(x) if((x)==0) asm volatile("bkpt 0")
#else
#define ASSERT(x) if((x)==0) emergency_write_crashdump( (char *)__FILE__, __LINE__, 0);
#endif

#endif /* MY_ASSERT_H_ */
