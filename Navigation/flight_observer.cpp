/** ***********************************************************************
 * @file		flight_observer.cpp
 * @brief		windspeed and vario update
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#include "system_configuration.h"
#include "flight_observer.h"

#define SQR(x) ((x)*(x))
#define SIN(x) sinf(x)

#define ONE_DIV_BY_GRAVITY_TIMES_2 0.0509684f
#define RECIP_GRAVITY 0.1094f

void flight_observer_t::update (
    const float3vector &gnss_velocity,
    const float3vector &gnss_acceleration,
    const float3vector &ahrs_acceleration,
    const float3vector &air_velocity,
    float GNSS_altitude,
    float pressure_altitude,
    float TAS
  )
{
  float3vector windspeed;
  windspeed[NORTH] = windspeed_averager_NORTH.respond( gnss_velocity.e[NORTH] - air_velocity.e[NORTH]);
  windspeed[EAST]  = windspeed_averager_EAST .respond( gnss_velocity.e[EAST]  - air_velocity.e[EAST]);
  windspeed[DOWN]  = 0.0;

  // non TEC compensated vario, negative if *climbing* !
  vario_uncompensated_GNSS = KalmanVario_GNSS.update ( GNSS_altitude, ahrs_acceleration.e[DOWN]);
  vario_uncompensated_pressure = KalmanVario_pressure.update ( pressure_altitude, ahrs_acceleration.e[DOWN]);
#if 1
  speed_compensation_TAS = kinetic_energy_differentiator.respond( TAS * TAS * ONE_DIV_BY_GRAVITY_TIMES_2);
#else
  speed_compensation_TAS = // patch misused to transport AHRS-based compensation
		  (
		      (gnss_velocity.e[NORTH] - windspeed.e[NORTH]) * ahrs_acceleration.e[NORTH] +
		      (gnss_velocity.e[EAST]  - windspeed.e[EAST])  * ahrs_acceleration.e[EAST] +
		      KalmanVario_pressure.get_x(KalmanVario_t::VARIO) * KalmanVario_pressure.get_x(KalmanVario_t::ACCELERATION_OBSERVED)
		   ) * RECIP_GRAVITY;
#endif
  float acc_north = gnss_acceleration.e[NORTH]; //acceleration_averager_NORTH.respond(gnss_acceleration.e[NORTH]);
  float acc_east  = gnss_acceleration.e[EAST]; //acceleration_averager_EAST.respond(gnss_acceleration.e[EAST]);
  speed_compensation_GNSS =
		  (
		      (gnss_velocity.e[NORTH] - windspeed.e[NORTH]) * acc_north +
		      (gnss_velocity.e[EAST]  - windspeed.e[EAST])  * acc_east +
		      KalmanVario_GNSS.get_x(KalmanVario_t::VARIO) * KalmanVario_GNSS.get_x(KalmanVario_t::ACCELERATION_OBSERVED)
		   ) * RECIP_GRAVITY;

  vario_averager_pressure.respond( speed_compensation_TAS - vario_uncompensated_pressure); // -> positive on positive energy gain
  vario_averager_GNSS.respond( speed_compensation_GNSS - vario_uncompensated_GNSS); // -> positive on positive energy gain
}

void flight_observer_t::reset(float pressure_altitude, float GNSS_altitude)
{
  KalmanVario_GNSS.reset( GNSS_altitude, -9.81f);
  KalmanVario_pressure.reset( pressure_altitude, -9.81f);
}
