/*
 * embedded_math.h
 *
 *  Created on: May 12, 2021
 *      Author: schaefer
 */

#ifndef INC_EMBEDDED_MATH_H_
#define INC_EMBEDDED_MATH_H_

#include "vsqrtf.h"
#include "asin_atan.h"

#define M_PI_F 3.14159265358979323846f
#define M_PI   3.14159265358979323846
#define NAN_F 0x40000000
#define ONE 1.0f
#define TWO 2.0f
#define HALF 0.5f
#define QUARTER 0.25f

#define SQR(x) ((x)*(x))
#define SQRT(x) VSQRTF(x)
#define COS(x) cosf(x) // arm_cos_f32(x)
#define SIN(x) sinf(x) // arm_sin_f32(x)
#define ASIN(x) my_asinf(x)
#define ATAN2(y, x) my_atan2f(y, x)


#endif /* INC_EMBEDDED_MATH_H_ */
