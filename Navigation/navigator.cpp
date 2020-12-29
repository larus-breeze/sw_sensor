#include <navigator.h>

// to be called at 100 Hz
void navigator_t::update_IMU (
    const float3vector &acc,
    const float3vector &mag,
    const float3vector &gyro)
{
  ins.update_diff_GNSS (
      gyro, acc,
      GNSS_acceleration,
      GNSS_heading);

  true_airspeed[NORTH] = ins.get_north () * TAS;
  true_airspeed[EAST]  = ins.get_east ()  * TAS;
  true_airspeed[DOWN]  = ins.get_down ()  * TAS; // todo: do we need this one ?

  flight_observer.update (
      GNSS_velocity,
      GNSS_acceleration,
      ins.get_acc (),
      true_airspeed,
      GNSS_altitude,
      TAS);
}

// to be called at 10 Hz
void navigator_t::update_GNSS (const coordinates_t &coordinates)
{
  GNSS_velocity 	= coordinates.velocity;
  GNSS_acceleration 	= coordinates.acceleration;
  GNSS_heading 		= coordinates.relPosHeading;
  GNSS_altitude 	= coordinates.position.e[DOWN]; // negative altitude
  GNSS_speed 		= coordinates.speed_motion;

  vario_integrator.update (flight_observer.get_vario_INS(),
			   ins.get_euler ().y,
			   ins.get_circling_state ());
}
