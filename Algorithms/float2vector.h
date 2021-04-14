/*
 * float2vector.h
 *
 *  Created on: Apr 14, 2021
 *      Author: schaefer
 *
 *     todo probably BUGGY
 */

#ifndef FLOAT2VECTOR_H_
#define FLOAT2VECTOR_H_

class float2vector
{
public:
  float2vector(void)
      : e{0}
    {}
  float2vector( int right)
      {
	e[0]=e[1]=(float)right;
      }

  operator float3vector ( void)
    {
    float3vector ret;
    ret.e[0]=e[0];
    ret.e[1]=e[1];
    ret.e[2]=0.0f; // 3rd element = zero
    return ret;
    }

  float2vector & operator *( float right)
  {
    e[0] *= right;
    e[1] *= right;
    return *this;
  }

  float2vector & operator +( const float2vector &right)
  {
    e[0] += right.e[0];
    e[1] += right.e[1];
    return *this;
  }

  float2vector & operator -( const float2vector &right)
  {
    e[0] -= right.e[0];
    e[1] -= right.e[1];
    return *this;
  }

  void set (const float3vector &right)
  {
    e[0]=right.e[0];
    e[1]=right.e[1];
  }

private:
  float e[2];
};

#endif /* FLOAT2VECTOR_H_ */
