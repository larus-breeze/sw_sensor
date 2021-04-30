/*
 * float3matrix.h
 *
 *  Created on: Mar 6, 2013
 *      Author: schaefer
 */

#ifndef FLOAT3MATRIX_H_
#define FLOAT3MATRIX_H_

#include "arm_math.h"
#include "matrix.h"
#include "float3vector.h"

//! matrix of 3 * 3 float values
class float3matrix: public matrix<float, 3>
{
//friend class float3vector;
public:
	float3matrix( float *init=0):
		matrix<float, 3>( init)
		{
			ami.numCols=3;
			ami.numRows=3;
			ami.pData=(float*)matrix<float, 3>::e;
		}
	void transpose( float3matrix &result);
	float3vector operator *( const float3vector& right) const;
	float3matrix operator *( const float3matrix& right) const;
private:
	arm_matrix_instance_f32 ami;
};

#endif /* FLOAT3MATRIX_H_ */
