/**
 * @file    my_assert.h
 * @brief   special ASSERT macro
 * @author  Dr. Klaus Schaefer klaus.schaefer@h-da.de
 */

#ifndef MY_ASSERT_H_
#define MY_ASSERT_H_

#if 1
#define ASSERT(x) if((x)==0) asm("bkpt 0")
#else
#define ASSERT(x) ((void)(x))
#endif

#endif /* MY_ASSERT_H_ */
