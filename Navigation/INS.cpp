/** ***********************************************************************
 * @file		INS.cpp
 * @brief		INS Implementation
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#include "system_configuration.h"
#include "INS.h"
#include "GNSS.h"

#define P_GAIN 0.03
#define I_GAIN 0.00006
#define H_GAIN 38.0
#define ALTI_DIFF 0.0f // 0.136 // antenna height difference D-KCOM

#if 1
void INS_type::attitude_setup( const float3vector & acceleration, const float3vector & induction)
{
	float3vector north, east, down;

	down  = acceleration;
	down.negate();
	north = induction;

	down.normalize();
	north.normalize();

	// setup world coordinate system
	east  = down.vector_multiply(north);
	east.normalize();
	north = east.vector_multiply(down);
	north.normalize();

	// create rotation matrix from unity direction vectors
	float fcoordinates[]=
	{
			north.e[0], north.e[1], north.e[2],
			 east.e[0],  east.e[1],  east.e[2],
			 down.e[0],  down.e[1],  down.e[2]
	};

	float3matrix coordinates( fcoordinates);
	attitude.from_rotation_matrix(coordinates);
	attitude.get_rotation_matrix( body2nav);
	body2nav.transpose( nav2body);
	euler = attitude;
}
#endif

#define CIRCLE_LIMIT 200
#define HIGH_TURN_RATE_SQUARED 0.02
#define LOW_TURN_RATE_SQUARED 0.005

circle_state_t INS_type::update_circling_state( const float3vector &gyro)
{
	float turn_rate=gyro.e[RIGHT]*gyro.e[RIGHT] + gyro.e[BOTTOM]*gyro.e[BOTTOM];

	if( circling_counter < CIRCLE_LIMIT)
	  if( turn_rate > HIGH_TURN_RATE_SQUARED)
	    ++circling_counter;

	if( circling_counter > 0)
	  if( turn_rate < LOW_TURN_RATE_SQUARED)
	    --circling_counter;

	if( circling_counter == 0)
	  circle_state = STRAIGHT_FLIGHT;
	else if ( circling_counter == CIRCLE_LIMIT)
	  circle_state = CIRCLING;
	else
	  circle_state = TRANSITION;

	return circle_state;
}

void INS_type::update( const float3vector &acc, const float3vector &gyro)
{
	attitude.rotate(
			gyro.e[ROLL] * Ts_div_2,
			gyro.e[NICK] * Ts_div_2 ,
			gyro.e[YAW]  * Ts_div_2 );

	attitude.normalize();

	attitude.get_rotation_matrix( body2nav);
	body2nav.transpose( nav2body);

	acceleration_nav_frame = body2nav * acc;
	euler=attitude;
}
#if 0
void INS_type::update_compass( float3vector &gyro, float3vector &acc, float3vector &mag)
{
  // fixme: algorithm unchecked, probably wrong
	float3vector nav_correction;
	float3vector gyro_correction;

	float3vector nav_acc;
	nav_acc = body2nav * acc;
	nav_acc.normalize();

	float3vector nav_induction;
	nav_induction = body2nav * mag;
	nav_induction[ DOWN] = 0.0f;
	nav_induction.normalize();

	nav_correction.e[NORTH] = -1.0 * nav_acc[RIGHT];
	nav_correction.e[RIGHT] =        nav_acc[FRONT];
	nav_correction.e[DOWN]  = -1.0 * nav_induction[RIGHT];

	switch( update_circling_state( gyro))
	{
	  case STRAIGHT_FLIGHT:
	  {
		nav_correction = nav_correction * 0.2; // TUNING FACTOR
		gyro_correction = nav2body * nav_correction;

	  }
	  break;
	  case CIRCLING:
	  {
	    gyro_correction = nav2body * nav_correction;

	    float roll_angle_estimate = my_atan2f( gyro.e[RIGHT] , gyro.e[DOWN]);

	    if( roll_angle_estimate > M_PI / 2.0f)
	      roll_angle_estimate = roll_angle_estimate - M_PI; //% NOT 2 * pi

	    if( roll_angle_estimate < M_PI / -2.0f)
		    roll_angle_estimate = 0.0f;

	    if( roll_angle_estimate > M_PI / 2.0f)
		    roll_angle_estimate = 0.0f;

            gyro_correction[FRONT] = (roll_angle_estimate - euler.r) * 0.2f ; 	// TUNING FACTOR
            gyro_correction[RIGHT] = -0.2 * euler.n; 				// TUNING FACTOR

	    for( unsigned i=0; i<3; ++i) // FLIP/FLOP controller
		gyro_correction[i] = (gyro_correction[i] > 0.0f) ? 0.1f : -0.1f;
	  }
	  break;
	  default:
	  {
		gyro_correction = {0};
	  }
	  break;
	}

	update( acc, gyro + gyro_correction);
}
#endif

void INS_type::update_compass(
		const float3vector &gyro,
		const float3vector &acc,
		const float3vector &mag,
	    const float3vector &GNSS_acceleration
		)
{
	  float3vector nav_acceleration = body2nav * acc;
	  float3vector nav_induction    = body2nav * mag;

	  nav_correction[NORTH] = - nav_acceleration.e[EAST]  + GNSS_acceleration.e[EAST];
	  nav_correction[EAST]  = + nav_acceleration.e[NORTH] - GNSS_acceleration.e[NORTH];
	  nav_correction[DOWN]  =   nav_induction[RIGHT] * H_GAIN;

	  gyro_correction = nav2body * nav_correction;
	  gyro_correction *= P_GAIN;

	  if (update_circling_state (gyro) == STRAIGHT_FLIGHT)
	    {
	      gyro_integrator += gyro_correction; // update integrator
	      gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
	      update (acc, gyro + gyro_correction);
	    }
	  else
	    {
	      // don't update integrator but use it
	      gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
	      update (acc, gyro + gyro_correction);
	    }

}
void INS_type::update_diff_GNSS( //!< rotate quaternion taking angular rate readings
    const float3vector &gyro, const float3vector &acc,
    const float3vector &GNSS_acceleration,
	float GNSS_heading)
{
	  float3vector nav_acceleration = body2nav * acc;

	  float heading_gnss_work = GNSS_heading + ALTI_DIFF * sinf (euler.r); // todo correct this patch
	  heading_gnss_work = heading_gnss_work - euler.y;

	  if( heading_gnss_work > M_PI) // map into { -PI PI}
		heading_gnss_work -= 2*M_PI;
	  if( heading_gnss_work < -M_PI)
	    heading_gnss_work += 2*M_PI;

	  if( heading_gnss_work > 1.0) // limit to +/- 1.0
		heading_gnss_work = 1.0;
	  else if ( heading_gnss_work < -1.0)
		heading_gnss_work = -1.0;

	  nav_correction[NORTH] = - nav_acceleration.e[EAST]  + GNSS_acceleration.e[EAST];
	  nav_correction[EAST]  = + nav_acceleration.e[NORTH] - GNSS_acceleration.e[NORTH];
	  nav_correction[DOWN]  =   heading_gnss_work * H_GAIN;

	  gyro_correction = nav2body * nav_correction;
	  gyro_correction *= P_GAIN;

	  if (update_circling_state (gyro) == STRAIGHT_FLIGHT)
	    {
	      gyro_integrator += gyro_correction; // update integrator
	      gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
	      update (acc, gyro + gyro_correction);
	    }
	  else
	    {
	      // don't update integrator but use it
	      gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
	      update (acc, gyro + gyro_correction);
	    }
}
