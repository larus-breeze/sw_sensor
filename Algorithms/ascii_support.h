/** ***********************************************************************
 * @file		ascii_support.h
 * @brief		Simple and fast ASCII converters
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef ASCII_SUPPORT_H_
#define ASCII_SUPPORT_H_

#include <stdint.h>

#ifdef __cplusplus

void utox(uint32_t value, char* result, uint8_t nibbles = 8);
void lutox(uint64_t value, char* result);

extern "C"
 {
#endif /* __cplusplus */

float string2float(char *input);
char* ftoa(float Value, char* Buffer);
char* itoa(int value, char* result, int base=10);

#ifdef __cplusplus
 }
#endif /* __cplusplus */

#endif /* ASCII_SUPPORT_H_ */
