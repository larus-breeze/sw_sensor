/***********************************************************************//**
 * @file		quaternion.hpp
 * @brief		quaternion implementation
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef QUATERNION_H
#define QUATERNION_H

#include "vector.h"
#include "float3matrix.h"
#include "float3vector.h"
#include "euler.h"
#include "asin_atan.h"
#include "arm_math.h"
#include "vsqrtf.h"

//! quaternion - special form of vector<4>
template <class datatype > class quaternion: public vector <datatype, 4>
{
public:
	//! constructor from eulerwinkel
	quaternion( vector <datatype, 3> &init)
	{
		from_euler( init[0], init[1], init[2]);
	}

	//! constructor from datatype[4]
	quaternion( )
	: vector <datatype, 4>({0})
	  {
	    vector <datatype, 4>::e[0]=1;
	  };

	//! normalize quaternion absolute value to 1.0
	inline void normalize(void)
	{
	  datatype tmp = vector<datatype, 4>::e[0] * vector<datatype, 4>::e[0]
		       + vector<datatype, 4>::e[1] * vector<datatype, 4>::e[1]
		       + vector<datatype, 4>::e[2] * vector<datatype, 4>::e[2]
		       + vector<datatype, 4>::e[3] * vector<datatype, 4>::e[3] ;
	  tmp = 1.0 / tmp;
	  tmp = VSQRTF( tmp);
	  for( int i = 0; i < 4; ++i)
		  vector<datatype, 4>::e[i] *= tmp;
	};

	//! quaternion -> euler angle transformation
	operator eulerangle <datatype> () const
	{
		datatype e0 = vector<datatype, 4>::e[0];
		datatype e1 = vector<datatype, 4>::e[1];
		datatype e2 = vector<datatype, 4>::e[2];
		datatype e3 = vector<datatype, 4>::e[3];

		eulerangle <datatype> _euler;

		//! formula from roenbaeck p34
		_euler.r = my_atan2f(  2.0 * (e0*e1 + e2*e3) , e0*e0 - e1*e1 - e2*e2 + e3*e3 );
		_euler.n = my_asinf(   2.0 * (e0*e2 - e3*e1));
		_euler.y = my_atan2f(  2.0 * (e0*e3 + e1*e2) , e0*e0 + e1*e1 - e2*e2 - e3*e3 );
		return _euler;
	}
	//! linearized euler component e0 (approximate nick angle / rad)
	inline datatype lin_e0( void) const
	{
		datatype e0 = vector<datatype, 4>::e[0];
		datatype e1 = vector<datatype, 4>::e[1];
		datatype e2 = vector<datatype, 4>::e[2];
		datatype e3 = vector<datatype, 4>::e[3];
		return 2.0 * (e0*e1 + e2*e3) / ( e0*e0 - e1*e1 - e2*e2 + e3*e3);

	}
	//! linearized euler component e1 (approximate roll angle / rad)
	inline datatype lin_e1( void) const
	{
		datatype e0 = vector<datatype, 4>::e[0];
		datatype e1 = vector<datatype, 4>::e[1];
		datatype e2 = vector<datatype, 4>::e[2];
		datatype e3 = vector<datatype, 4>::e[3];
		return 2.0f * (e0*e2 - e3*e1);
	}
	//! euler component e2
	inline datatype get_e2( void) const
	{
		datatype e0 = vector<datatype, 4>::e[0];
		datatype e1 = vector<datatype, 4>::e[1];
		datatype e2 = vector<datatype, 4>::e[2];
		datatype e3 = vector<datatype, 4>::e[3];
		return my_atan2f(  2.0 * (e0*e3 + e1*e2) , e0*e0 + e1*e1 - e2*e2 - e3*e3 );
	}

	//! get north component of attitude
	inline datatype get_north( void) const
	{
		datatype e0 = vector<datatype, 4>::e[0];
		datatype e1 = vector<datatype, 4>::e[1];
		datatype e2 = vector<datatype, 4>::e[2];
		datatype e3 = vector<datatype, 4>::e[3];
		return e0*e0 + e1*e1 - e2*e2 - e3*e3;
	}

	//! get east component of attitude
	inline datatype get_east( void) const
	{
		datatype e0 = vector<datatype, 4>::e[0];
		datatype e1 = vector<datatype, 4>::e[1];
		datatype e2 = vector<datatype, 4>::e[2];
		datatype e3 = vector<datatype, 4>::e[3];
		return 2.0 * (e0*e3 + e1*e2);
	}

	//! get down component of attitude
	inline datatype get_down( void) const
	{
		datatype e0 = this->e[0];
		datatype e1 = this->e[1];
		datatype e2 = this->e[2];
		datatype e3 = this->e[3];
		return 2.0 * (e1*e3-e0*e2);
	}

	//! quaternion update using rotation vector
	void rotate( datatype p, datatype q, datatype r)
	{
		datatype e0 = vector<datatype, 4>::e[0];
		datatype e1 = vector<datatype, 4>::e[1];
		datatype e2 = vector<datatype, 4>::e[2];
		datatype e3 = vector<datatype, 4>::e[3];

		//! R.Rogers formula 2.92
		vector<datatype, 4>::e[0] += (-e1*p -e2*q -e3*r);
		vector<datatype, 4>::e[1] += ( e0*p +e2*r -e3*q);
		vector<datatype, 4>::e[2] += ( e0*q -e1*r +e3*p);
		vector<datatype, 4>::e[3] += ( e0*r +e1*q -e2*p);

		normalize();
	}

	//! euler angle -> quaternion transformation
	void from_euler( datatype p, datatype q, datatype r)
	{
		p *= 0.5; q *= 0.5; r *= 0.5;
		register datatype sinphi   = sinf( p);
		register datatype cosphi   = cosf( p);
		register datatype sintheta = sinf( q);
		register datatype costheta = cosf( q);
		register datatype sinpsi   = sinf( r);
		register datatype cospsi   = cosf( r);

		vector<datatype, 4>::e[0] = cosphi*costheta*cospsi + sinphi*sintheta*sinpsi;
		vector<datatype, 4>::e[1] = sinphi*costheta*cospsi + cosphi*sintheta*sinpsi;
		vector<datatype, 4>::e[2] = cosphi*sintheta*cospsi + sinphi*costheta*sinpsi;
		vector<datatype, 4>::e[3] = cosphi*costheta*sinpsi - sinphi*sintheta*cospsi;
	}

	//! quaternion -> rotation matrix transformation
	void get_rotation_matrix (matrix<datatype, 3> &m) const
	{
		//! R.Rogers formula 2.90
		datatype e0=vector<datatype, 4>::e[0];
		datatype e1=vector<datatype, 4>::e[1];
		datatype e2=vector<datatype, 4>::e[2];
		datatype e3=vector<datatype, 4>::e[3];

	    m.e[0][0] = 2.0 * (e0*e0+e1*e1) - 1.0;
	    m.e[0][1] = 2.0 * (e1*e2-e0*e3);
	    m.e[0][2] = 2.0 * (e1*e3+e0*e2);

	    m.e[1][0] = 2.0 * (e1*e2+e0*e3);
	    m.e[1][1] = 2.0 * (e0*e0+e2*e2) - 1.0;
	    m.e[1][2] = 2.0 * (e2*e3-e0*e1);

	    m.e[2][0] = 2.0 * (e1*e3-e0*e2);
	    m.e[2][1] = 2.0 * (e2*e3+e0*e1);
	    m.e[2][2] = 2.0 * (e0*e0+e3*e3) - 1.0;
	}
	quaternion <datatype> operator * ( quaternion <datatype> & right) //!< quaternion multiplication
		{
		quaternion <datatype> result;
		datatype e0=vector<datatype, 4>::e[0];
		datatype e1=vector<datatype, 4>::e[1];
		datatype e2=vector<datatype, 4>::e[2];
		datatype e3=vector<datatype, 4>::e[3];
		datatype re0=right.vector<datatype, 4>::e[0];
		datatype re1=right.vector<datatype, 4>::e[1];
		datatype re2=right.vector<datatype, 4>::e[2];
		datatype re3=right.vector<datatype, 4>::e[3];
		result.vector<datatype, 4>::e[0] = e0 * re0 - e1 * re1 - e2 * re2 + e3 * re3;
		result.vector<datatype, 4>::e[1] = e1 * re0 + e2 * re3 - e3 * re2 + e0 * re1;
		result.vector<datatype, 4>::e[2] = e3 * re1 + e0 * re2 - e1 * re3 + e2 * re0;
		result.vector<datatype, 4>::e[3] = e1 * re2 - e2 * re1 + e0 * re3 + e3 * re0;
		return result;
		}

	void from_rotation_matrix( matrix<datatype, 3> &rotm) //!< rotation matrix -> quaternion transformation
		{
		float tmp;
		tmp = 1.0f + rotm.e[0][0] + rotm.e[1][1] + rotm.e[2][2];
		//! formula from roenbaeck p35
		assert(( tmp > 0.0f) && (tmp != 0.0f/0.0f /* NAN */));
		tmp = VSQRTF( tmp);
		tmp *= 0.5;
		vector<datatype, 4>::e[0] = tmp;
		tmp = 0.25f / tmp;
		vector<datatype, 4>::e[1] = tmp * (rotm.e[2][1] - rotm.e[1][2]);
		vector<datatype, 4>::e[2] = tmp * (rotm.e[0][2] - rotm.e[2][0]);
		vector<datatype, 4>::e[3] = tmp * (rotm.e[1][0] - rotm.e[0][1]);
		normalize(); // compensate computational inaccuracies
		};
};

#endif // QUATERNION_H
