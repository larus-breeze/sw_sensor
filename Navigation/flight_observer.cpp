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
    float altitude,
    float TAS
  )
{
  float3vector windspeed;
  windspeed[NORTH] = windspeed_averager_NORTH.respond( gnss_velocity.e[NORTH] - air_velocity.e[NORTH]);
  windspeed[EAST]  = windspeed_averager_EAST .respond( gnss_velocity.e[EAST]  - air_velocity.e[EAST]);
  windspeed[DOWN]  = 0.0;

  // non TEC compensated vario, negative if *climbing* !
  vario_uncompensated = KalmanVario.update ( altitude, ahrs_acceleration.e[DOWN]);

  effective_vertical_acceleration = KalmanVario_t::ACCELERATION_OBSERVED;

  //  speed_compensation_TAS = kinetic_energy_differentiator.respond( TAS * TAS * ONE_DIV_BY_GRAVITY_TIMES_2);

  speed_compensation_TAS = // patch misused to transport AHRS-based compensation
		  (
		      (gnss_velocity.e[NORTH] - windspeed.e[NORTH]) * ahrs_acceleration.e[NORTH] +
		      (gnss_velocity.e[EAST]  - windspeed.e[EAST])  * ahrs_acceleration.e[EAST] +
		      KalmanVario.get_x(KalmanVario_t::VARIO) * KalmanVario.get_x(KalmanVario_t::ACCELERATION_OBSERVED)
		   ) * RECIP_GRAVITY;

  speed_compensation_INS =
		  (
		      (gnss_velocity.e[NORTH] - windspeed.e[NORTH]) * gnss_acceleration.e[NORTH] +
		      (gnss_velocity.e[EAST]  - windspeed.e[EAST])  * gnss_acceleration.e[EAST] +
		      KalmanVario.get_x(KalmanVario_t::VARIO) * KalmanVario.get_x(KalmanVario_t::ACCELERATION_OBSERVED)
		   ) * RECIP_GRAVITY;

  vario_averager_TAS.respond( speed_compensation_TAS - vario_uncompensated); // -> positive on positive energy gain
  vario_averager_INS.respond( speed_compensation_INS - vario_uncompensated); // -> positive on positive energy gain
}
