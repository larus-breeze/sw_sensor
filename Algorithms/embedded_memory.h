/** ***********************************************************************
 * @file		embedded_memory.h
 * @brief		COMMON data declarations
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef EMBEDDED_MEMORY_H_
#define EMBEDDED_MEMORY_H_

#define COMMON __attribute__ ((section ("common_data")))
#define CONSTEXPR_ROM constexpr __attribute__ ((section (".rodata")))
#ifndef ROM
#define ROM const __attribute__ ((section (".rodata")))
#endif
#endif /* EMBEDDED_MEMORY_H_ */
