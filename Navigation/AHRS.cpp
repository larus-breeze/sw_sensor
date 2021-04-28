/** ***********************************************************************
 * @file		AHRS.cpp
 * @brief		AHRS Implementation: maintain aircraft attitude
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#include <AHRS.h>
#include "system_configuration.h"
#include "my_assert.h"
#include "GNSS.h"
#include "embedded_memory.h"

#define P_GAIN 0.03f			//!< Attitude controller: proportional gain
#define I_GAIN 0.00006f 		//!< Attitude controller: integral gain
#define H_GAIN 38.0f			//!< Attitude controller: horizontal gain
#define M_H_GAIN -10.0f			//!< Attitude controller: horizontal gain magnetic
#define CROSS_GAIN 0.05f		//!< Attitude controller: cross-product gain

#define HIGH_TURN_RATE 0.15 		//!< turn rate high limit
#define LOW_TURN_RATE  0.0707 		//!< turn rate low limit

COMMON compass_calibration_t *p_compass_calibration=0;

/**
 * @brief initial attitude setup from observables
 */
void
AHRS_type::attitude_setup (const float3vector &acceleration,
			   const float3vector &induction)
{
  float3vector north, east, down;

  down = acceleration;
  down.negate ();
  north = induction;

  down.normalize ();
  north.normalize ();

  // setup world coordinate system
  east = down.vector_multiply (north);
  east.normalize ();
  north = east.vector_multiply (down);
  north.normalize ();

  // create rotation matrix from unity direction vectors
  float fcoordinates[] =
    { north.e[0], north.e[1], north.e[2], east.e[0], east.e[1], east.e[2],
	down.e[0], down.e[1], down.e[2] };

  float3matrix coordinates (fcoordinates);
  attitude.from_rotation_matrix (coordinates);
  attitude.get_rotation_matrix (body2nav);
  body2nav.transpose (nav2body);
  euler = attitude;
}

/**
 * @brief  decide about circling state
 */
circle_state_t
AHRS_type::update_circling_state (const float3vector &gyro)
{
  float turn_rate_abs = abs (turn_rate);

  if (circling_counter < CIRCLE_LIMIT)
    if (turn_rate_abs > HIGH_TURN_RATE)
      ++circling_counter;

  if (circling_counter > 0)
    if (turn_rate_abs < LOW_TURN_RATE)
      --circling_counter;

  if (circling_counter == 0)
    circle_state = STRAIGHT_FLIGHT;
  else if (circling_counter == CIRCLE_LIMIT)
    circle_state = CIRCLING;
  else
    circle_state = TRANSITION;

  return circle_state;
}

void
AHRS_type::feed_compass_calibration (const float3vector &mag)
{
  float3vector expected_induction = nav2body * NAV_INDUCTION;

  for (unsigned i = 0; i < 3; ++i)
    mag_calibrator[i].add_value (expected_induction.e[i], mag.e[i]);
}

/**
 * @brief  generic update of AHRS
 *
 * Side-effect: create rotation matrices, NAV-acceleration, NAV-induction
 */
void
AHRS_type::update (const float3vector &acc, const float3vector &gyro,
		   const float3vector &mag)
{
  attitude.rotate (gyro.e[ROLL] * Ts_div_2, gyro.e[NICK] * Ts_div_2,
		   gyro.e[YAW] * Ts_div_2);

  attitude.normalize ();

  attitude.get_rotation_matrix (body2nav);
  body2nav.transpose (nav2body);

  acceleration_nav_frame = body2nav * acc;
  induction_nav_frame = body2nav * mag;
  euler = attitude;

  float3vector nav_rotation;
  nav_rotation = body2nav * gyro;
  turn_rate = nav_rotation[DOWN];

  slip_angle_averager.respond (my_atan2f (-acc.e[RIGHT], -acc.e[DOWN]));
  // todo keine ahnung warum da PI dazu muß !
  nick_angle_averager.respond (my_atan2f (acc.e[FRONT], acc.e[DOWN]) - 3.1416f);
}

/**
 * @brief  update attitude from IMU data D-GNSS compass
 */
void
AHRS_type::update_diff_GNSS (const float3vector &gyro, const float3vector &acc,
			     const float3vector &mag,
			     const float3vector &GNSS_acceleration,
			     float GNSS_heading)
{
  float3vector nav_acceleration = body2nav * acc;

  float heading_gnss_work = GNSS_heading
      + ALTI_DIFF * sinf (euler.r) - HORIZ_DIFF;
  heading_gnss_work = heading_gnss_work - euler.y;

  if (heading_gnss_work > M_PI) // map into { -PI PI}
    heading_gnss_work -= 2 * M_PI;
  if (heading_gnss_work < -M_PI)
    heading_gnss_work += 2 * M_PI;

  if (heading_gnss_work > 1.0) // limit to +/- 1.0
    heading_gnss_work = 1.0;
  else if (heading_gnss_work < -1.0)
    heading_gnss_work = -1.0;

  nav_correction[NORTH] = -nav_acceleration.e[EAST] + GNSS_acceleration.e[EAST];
  nav_correction[EAST] = +nav_acceleration.e[NORTH]
      - GNSS_acceleration.e[NORTH];
  nav_correction[DOWN] = heading_gnss_work * H_GAIN;

  gyro_correction = nav2body * nav_correction;
  gyro_correction *= P_GAIN;

  circle_state_t old_circle_state = circle_state;
  update_circling_state (gyro);

  if (circle_state == STRAIGHT_FLIGHT)
    {
      gyro_integrator += gyro_correction; // update integrator
      gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
    }
  else
    {
      // don't update integrator but use it
      gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
    }

  update (acc, gyro + gyro_correction, mag);


  if (circle_state == CIRCLING) // only here we get fresh magnetic information
    {
      if( old_circle_state == TRANSITION)
	  for (unsigned i = 0; i < 3; ++i)
	      mag_calibrator[i].reset();

      feed_compass_calibration (mag);
    }
  else if (old_circle_state == CIRCLING) // coming out of circling
    {
      compass_calibration.set_calibration( mag_calibrator);
      p_compass_calibration = &compass_calibration;
    }
}

