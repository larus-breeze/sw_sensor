#ifndef FLIGHT_OBSERVER_H_
#define FLIGHT_OBSERVER_H_

#include "GNSS.h"
#include <differentiator.h>
#include "KalmanVario.h"

#define SQR(x) ((x)*(x))
#define SIN(x) sinf(x)
#include "pt2.h"

#define WINDSPEED_F_BY_FS ( 1.0f / 2.0f / 30.0f / 100.0f) // 30s
#define VARIO_F_BY_FS ( 1.0f / 2.0f / 1.0f / 100.0f) // 1s

class flight_observer_t
{
public:
  flight_observer_t( void)
  :
  vario_averager_TAS( VARIO_F_BY_FS),
  vario_averager_INS( VARIO_F_BY_FS),
  windspeed_averager_NORTH( WINDSPEED_F_BY_FS),
  windspeed_averager_EAST( WINDSPEED_F_BY_FS),
  kinetic_energy_differentiator( 1.0f, 1.0f / 100.0f)
  {
  };
	void update
	(
	    const float3vector &gnss_velocity,
	    const float3vector &gnss_acceleration,
	    const float3vector &ahrs_acceleration,
	    const float3vector &air_velocity,
	    float altitude,
	    float TAS
	);
	float get_pressure_altitude( void) const;
	float get_speed_compensation_TAS( void ) const
	{
		return speed_compensation_TAS;
	}
	float get_speed_compensation_INS( void ) const
	{
		return speed_compensation_INS;
	}
	float get_vario_uncompensated( void ) const
	{
		return vario_uncompensated;
	}
	float get_vario_TAS( void ) const
	{
		return (float)( vario_averager_TAS.get_output());
	}
	float get_vario_INS( void ) const
	{
		return vario_averager_INS.get_output();
	}
	float3vector get_wind( void ) const
	{
		float3vector retv;
		retv[NORTH] = windspeed_averager_NORTH.get_output();
		retv[EAST]  = windspeed_averager_EAST.get_output();
		retv[DOWN]  = 0.0f;
		return retv;
	}
	float get_eff_vert_acc( void) const
	{
		return KalmanVario.get_x( KalmanVario_t::ACCELERATION_OBSERVED);
	}
private:
	pt2<float,float> vario_averager_TAS;
	pt2<float,float> vario_averager_INS;
	pt2<float,float> windspeed_averager_NORTH;
	pt2<float,float> windspeed_averager_EAST;
	differentiator<float,float>kinetic_energy_differentiator;
	float speed_compensation_TAS;
	float speed_compensation_INS;
	float vario_uncompensated;
	KalmanVario_t KalmanVario;
};

#endif /* FLIGHT_OBSERVER_H_ */
