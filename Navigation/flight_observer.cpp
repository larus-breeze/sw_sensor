/** ***********************************************************************
 * @file		flight_observer.cpp
 * @brief		windspeed and vario update
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#include "system_configuration.h"
#include "flight_observer.h"

#define SQR(x) ((x)*(x))
#define SIN(x) arm_sin_f32(x)

#define ONE_DIV_BY_GRAVITY_TIMES_2 0.0509684f
#define RECIP_GRAVITY 0.1094f

void flight_observer_t::update (
    const float3vector &gnss_velocity,
    const float3vector &gnss_acceleration,
    const float3vector &ahrs_acceleration,
    const float3vector &air_velocity,
    const float3vector &observed_wind, // smart long-term average
    float GNSS_altitude,
    float pressure_altitude,
    float TAS
  )
{
  float3vector windspeed; // short-term average
  windspeed[NORTH] = windspeed_averager_NORTH.respond( gnss_velocity.e[NORTH] - air_velocity.e[NORTH]);
  windspeed[EAST]  = windspeed_averager_EAST .respond( gnss_velocity.e[EAST]  - air_velocity.e[EAST]);
  windspeed[DOWN]  = 0.0;

  // non TEC compensated vario, negative if *climbing* !
  vario_uncompensated_GNSS = KalmanVario_GNSS.update ( GNSS_altitude, ahrs_acceleration.e[DOWN]);
  vario_uncompensated_pressure = KalmanVario_pressure.update ( pressure_altitude, ahrs_acceleration.e[DOWN]);

  speed_compensation_TAS = kinetic_energy_differentiator.respond( TAS * TAS * ONE_DIV_BY_GRAVITY_TIMES_2);

  float acc_north = gnss_acceleration.e[NORTH]; //todo: check: acceleration_averager_NORTH.respond(gnss_acceleration.e[NORTH]);
  float acc_east  = gnss_acceleration.e[EAST]; //acceleration_averager_EAST.respond(gnss_acceleration.e[EAST]);

  speed_compensation_GNSS =
		  (
		      (gnss_velocity.e[NORTH] - observed_wind.e[NORTH]) * acc_north +
		      (gnss_velocity.e[EAST]  - observed_wind.e[EAST])  * acc_east +
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
