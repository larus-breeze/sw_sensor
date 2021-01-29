/*
 * vsqrtf.h
 *
 *  Created on: Jan 11, 2018
 *      Author: schaefer
 */

#ifndef VSQRTF_H_
#define VSQRTF_H_

#if 1 // in arm_math.h

__attribute__((always_inline)) static float inline VSQRTF(float op1)
  {
  	float result;
  	asm volatile ("vsqrt.f32 %0, %1" : "=&w" (result) : "w" (op1) );
  	return (result);
  }

#endif

#endif /* VSQRTF_H_ */
