/** ***********************************************************************
 * @file		navigator.cpp
* @brief		maintain speed attitude and position data
  * @author		Dr. Klaus Schaefer
 **************************************************************************/

#include <navigator.h>

// to be called at 100 Hz
void navigator_t::update_IMU (
    const float3vector &acc,
    const float3vector &mag,
    const float3vector &gyro)
{
  ahrs.update( gyro, acc, mag,
	    GNSS_acceleration,
	    GNSS_heading);

  ahrs_magnetic.update_compass(
	  gyro, acc, mag,
	  GNSS_acceleration);

  float3vector heading_vector;
  heading_vector[NORTH] = ahrs.get_north (); // todo which special ins to use = ???
  heading_vector[EAST]  = ahrs.get_east  ();
  heading_vector[DOWN]  = ahrs.get_down  (); // todo: do we need this one ?

  flight_observer.update (
      GNSS_velocity,
      GNSS_acceleration,
      ahrs.get_nav_acceleration (),
      heading_vector,
      GNSS_altitude,
      atmosphere.get_altitude(),
      TAS,
      ahrs.get_circling_state(),
      wind_average_observer.get_value()
      );
}

// to be called at 10 Hz
void navigator_t::update_GNSS (const coordinates_t &coordinates)
{
  if( isnan( coordinates.acceleration.e[NORTH])) // presently no GNSS fix
    {
      return; // todo needs to be improved
    }
  GNSS_velocity 	= coordinates.velocity;
  GNSS_acceleration	= coordinates.acceleration;
  GNSS_heading 		= coordinates.relPosHeading;
  GNSS_altitude 	= coordinates.position.e[DOWN]; // negative altitude
  GNSS_speed 		= coordinates.speed_motion;

  wind_average_observer.update( flight_observer.get_instant_wind(), // do this here because of the update rate 10Hz
				ahrs.get_euler ().y,
				ahrs.get_circling_state ());

  float3vector relative_wind_NAV = flight_observer.get_instant_wind() - wind_average_observer.get_value();
  float3vector relative_wind_BODY =  ahrs.get_nav2body() * relative_wind_NAV;
  relative_wind_observer.update(relative_wind_BODY,
				ahrs.get_euler ().y,
				ahrs.get_circling_state ());

#if N_PROBES == 5
  probe[1] = relative_wind_observer.get_value().e[FRONT];
  probe[2] = relative_wind_observer.get_value().e[RIGHT] / COS(ahrs.get_euler().r);
  float3vector wind_correction_nav = ahrs.get_body2nav() * relative_wind_observer.get_value();
  float3vector instant_wind_corrected = flight_observer.get_instant_wind() - wind_correction_nav;
  probe[3] = instant_wind_corrected.e[NORTH];
  probe[4] = instant_wind_corrected.e[EAST];
#endif
  vario_integrator.update (flight_observer.get_vario_GNSS(), // here because of the update rate 10Hz
			   ahrs.get_euler ().y,
			   ahrs.get_circling_state ());
}

void navigator_t::report_data(output_data_t &d)
{
    d.TAS 			= TAS;
    d.IAS 			= IAS;

    d.euler			= ahrs.get_euler();
    d.q				= ahrs.get_attitude();

    d.euler_magnetic		= ahrs_magnetic.get_euler();
    d.q_magnetic		= ahrs_magnetic.get_attitude();

#if USE_GNSS_VARIO
    d.vario			= flight_observer.get_vario_GNSS(); // todo pick one vario
#else
    d.vario			= flight_observer.get_vario_pressure();
#endif
    d.vario_pressure		= flight_observer.get_vario_pressure();
    d.integrator_vario		= vario_integrator.get_value();
    d.vario_uncompensated 	= flight_observer.get_vario_uncompensated_GNSS();

    d.wind			= flight_observer.get_instant_wind(); // short-term avg
    d.wind_average		= wind_average_observer.get_value();  // smart long-term avg

    d.speed_compensation_TAS 	= flight_observer.get_speed_compensation_TAS();
    d.speed_compensation_INS 	= flight_observer.get_speed_compensation_INS();
    d.effective_vertical_acceleration
				= flight_observer.get_effective_vertical_acceleration();

    d.circle_mode 		= ahrs.get_circling_state();
    d.nav_correction		= ahrs.get_nav_correction();
    d.gyro_correction		= ahrs.get_gyro_correction();
    d.nav_acceleration_gnss 	= ahrs.get_nav_acceleration();
    d.nav_acceleration_mag 	= ahrs_magnetic.get_nav_acceleration();
    d.nav_induction_gnss 	= ahrs.get_nav_induction();
    d.nav_induction_mag 	= ahrs_magnetic.get_nav_induction();

    d.turn_rate			= ahrs.get_turn_rate();
    d.slip_angle		= ahrs.getSlipAngle();
    d.nick_angle		= ahrs.getNickAngle();
}

//! eventually make magnetic calibration permanent
void navigator_t::handle_magnetic_calibration (void) const
{
  ahrs.handle_magnetic_calibration(); // todo: eventually do this with the magnetic AHRS
}
