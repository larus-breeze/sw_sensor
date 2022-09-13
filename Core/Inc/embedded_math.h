/**
 * @file    embedded_math.h
 * @brief   settings for special embedded target
 * @author  Dr. Klaus Schaefer dr.klaus.schaefer@mail.de
 */
#ifndef INC_EMBEDDED_MATH_H_
#define INC_EMBEDDED_MATH_H_

#include "arm_math.h"
#include "my_asin.h"

typedef float ftype;

#define M_PI_F 3.14159265358979323846f
#define M_PI   3.14159265358979323846

#define ZERO 0.0f
#define ONE 1.0f
#define TWO 2.0f
#define HALF 0.5f
#define QUARTER 0.25f

#define SQR(x) ((x)*(x))
#define SQRT(x) VSQRTF(x)
#define COS(x) arm_cos_f32(x)
#define SIN(x) arm_sin_f32(x)
#define ASIN(x) my_asinf(x)
inline float ATAN2( float y, float x)
{
	float result;
	(void)arm_atan2_f32( y, x, &result);
	return result;
}

#define CLIP( x, min, max) ((x) < (min) ? (min) : (x) > (max) ? (max) : (x))

__attribute__((always_inline)) static float inline VSQRTF(float op1)
  {
  	float result;
  	asm volatile ("vsqrt.f32 %0, %1" : "=&w" (result) : "w" (op1) );
  	return (result);
  }

#endif /* INC_EMBEDDED_MATH_H_ */
