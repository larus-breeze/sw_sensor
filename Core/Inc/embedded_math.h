/**
 * @file	embedded_math.h
 * @brief   	settings for special embedded target
 * @author	Dr. Klaus Schaefer
 * @copyright 	Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/

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

__attribute__((always_inline)) static float inline VSQRTF(float op1)
  {
  	float result;
  	asm volatile ("vsqrt.f32 %0, %1" : "=&w" (result) : "w" (op1) );
  	return (result);
  }

#ifdef __cplusplus

template <typename type> type CLIP( type x, type min, type max)
{
  return (x < min) ? min : (x > max) ? max : x;
}

#endif // cplusplus
#endif /* INC_EMBEDDED_MATH_H_ */
