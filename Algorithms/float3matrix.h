/*
 * float3matrix.h
 *
 *  Created on: Mar 6, 2013
 *      Author: schaefer
 */

#ifndef FLOAT3MATRIX_H_
#define FLOAT3MATRIX_H_

#include "embedded_math.h"
#include "matrix.h"
#include "float3vector.h"

//! matrix of 3 * 3 float values
class float3matrix: public matrix<float, 3>
{
//friend class float3vector;
public:
	float3matrix( const float *init=0):
		matrix<float, 3>( init)
		{
			ami.numCols=3;
			ami.numRows=3;
			ami.pData=(float*)matrix<float, 3>::e;
		}
	float3matrix( const float init[3][3]):
		matrix<float, 3>( init)
		{
			ami.numCols=3;
			ami.numRows=3;
			ami.pData=(float*)matrix<float, 3>::e;
		}
	void transpose( float3matrix &result);
	float3vector operator *( const float3vector& right) const;
	float3matrix operator *( const float3matrix& right) const;
	float3vector reverse_map( const float3vector& right) const
        {
                float3vector ret;
                for( unsigned k=0; k<3; ++k)
                {
                        ret[k]=ZERO;
                        for( unsigned i=0; i<3; ++i)
                        {	// multiply by transposed matrix
                                ret[k] += matrix<float, 3>::e[i][k] * right.e[i];
                        }
                }
                return ret;
        }

private:
	arm_matrix_instance_f32 ami;
};

#endif /* FLOAT3MATRIX_H_ */
