 /** ***********************************************************************
 * @file		float3matrix.cpp
 * @brief		float3matrix implementation
 *
 * Handles calling of DSP library functions
 *
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#include "my_assert.h"
#include "float3matrix.h"
#include "embedded_math.h"

void float3matrix::transpose( float3matrix &result)
{
	arm_matrix_instance_f32 from, to;
	from.numCols=3;
	from.numRows=3;
	from.pData=(float *)(this->e);
	to.numCols=3;
	to.numRows=3;
	to.pData=(float *)(&(result.e));
	(void)arm_mat_trans_f32( &from, &to);
}
#if 0
float3vector float3matrix::operator *( const float3vector& right) const
{
	float3vector ret;
	arm_matrix_instance_f32 ret_ami;
	ret_ami.numCols=1;
	ret_ami.numRows=3;
	ret_ami.pData=ret.e;
	arm_matrix_instance_f32 input;
	input.numCols=1;
	input.numRows=3;
	input.pData=(float *)(right.e);
	(void)arm_mat_mult_f32( &ami, &input, &ret_ami);
	return ret;
}
#endif
float3matrix float3matrix::operator *( const float3matrix& right) const
{
	float3matrix ret;
	arm_matrix_instance_f32 ret_ami;
	ret_ami.numCols=3;
	ret_ami.numRows=3;
	ret_ami.pData=(float32_t *)(ret.e);
	arm_matrix_instance_f32 input;
	input.numCols=3;
	input.numRows=3;
	input.pData=(float *)(right.e);
	(void)arm_mat_mult_f32( &ami, &input, &ret_ami);
	return ret;
}
