/** ***********************************************************************
 * @file		navigator.cpp
 * @brief		maintain speed attitude and position data
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#ifndef NAVIGATORT_H_
#define NAVIGATORT_H_

#include <system_configuration.h>
#include <AHRS.h>
#include "GNSS.h"
#include "differentiator.h"
#include "smart_averager.h"
#include "atmosphere.h"
#include "flight_observer.h"
#include "data_structures.h"

class navigator_t
{
public:
  navigator_t (void)
	:ahrs (0.01f),
	 ahrs_magnetic (0.01f),
	 atmosphere (101325.0f),
	 vario_integrator( configuration( VARIO_INT_TC)),
	 wind_average_observer( configuration( MEAN_WIND_TC))
  {
    GNSS_heading = NAN;
  };

  void report_data( output_data_t &d);
  void handle_magnetic_calibration( void) const;
  void set_from_euler ( float r, float n, float y)
  {
    ahrs.set_from_euler(r, n, y);
  }
  /**
   * @brief update absolute pressure
   * called @ 100 Hz
   */
  void update_pabs( float pressure)
  {
    atmosphere.set_pressure(pressure);
  }

  void reset_altitude( void)
  {
    flight_observer.reset( atmosphere.get_altitude(), GNSS_altitude);
  }
  /**
   * @brief update pitot pressure
   * called @ 100 Hz
   */
  void update_pitot( float pressure)
  {
    pitot_pressure=pressure;
    TAS = atmosphere.get_TAS_from_dynamic_pressure ( pitot_pressure);
    IAS = atmosphere.get_IAS_from_dynamic_pressure ( pitot_pressure);
  }
  /**
   * @brief update AHRS from IMU
   * called @ 100 Hz, triggers all fast calculations
   */
  void update_IMU( const float3vector &acc, const float3vector &mag, const float3vector &gyro);
  /**
   * @brief update navigation GNSS
   * called @ 10 Hz
   */

  void update_GNSS( const coordinates_t &coordinates /* , const float3vector & _GNSS_acceleration*/);

  /**
   * @brief return aggregate flight observer
   */
  const flight_observer_t &get_flight_observer( void) const
    {
    return flight_observer;
    }

  void set_attitude( float roll, float nick, float yaw)
  {
    ahrs.set_from_euler(roll, nick, yaw);
    ahrs_magnetic.set_from_euler(roll, nick, yaw);
  }
  float get_IAS( void) const
  {
    return IAS;
  }

private:
  AHRS_type 	ahrs;
  AHRS_type	ahrs_magnetic;

  atmosphere_t 	atmosphere;
  float 	pitot_pressure;
  float 	TAS;
  float 	IAS;
  float3vector 	GNSS_velocity;
  float		GNSS_speed;
  float3vector 	GNSS_acceleration;
  float 	GNSS_heading;
  float 	GNSS_altitude;

  flight_observer_t 	flight_observer;
  smart_averager< float> 	vario_integrator;
  smart_averager< float3vector, true> wind_average_observer; // configure wind average clamping on first circle
};

#endif /* NAVIGATORT_H_ */
