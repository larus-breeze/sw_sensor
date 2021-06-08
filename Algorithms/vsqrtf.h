/**
 * @file    vsqrtf.h
 * @brief   macro to force usage of "vsqrt" FPU instruction
 * @author  Dr. Klaus Schaefer dr.klaus.schaefer@mail.de
 */
#ifndef VSQRTF_H_
#define VSQRTF_H_

__attribute__((always_inline)) static float inline VSQRTF(float op1)
  {
  	float result;
  	asm volatile ("vsqrt.f32 %0, %1" : "=&w" (result) : "w" (op1) );
  	return (result);
  }

#endif /* VSQRTF_H_ */
