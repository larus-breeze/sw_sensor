#ifndef NAVIGATORT_H_
#define NAVIGATORT_H_

#include "GNSS.h"
#include "INS.h"
#include "differentiator.h"
#include "variointegrator.h"
#include "atmosphere.h"
#include "flight_observer.h"
#include "data_structures.h"

class navigator_t
{
public:
  navigator_t (void)
	:ins (0.01),
	 ins_magnetic (0.01),
     atmosphere (101325.0f)
  {};

  void report_data( output_data_t &d);

  void set_from_euler ( float r, float n, float y)
  {
    ins.set_from_euler(r, n, y);
  }
  /**
   * @brief update absolute pressure
   * called @ 100 Hz
   */
  void update_pabs( float pressure)
  {
    atmosphere.set_pressure(pressure);
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
  void update_GNSS_old( const coordinates_t &coordinates , float3vector acceleration);

  /**
   * @brief return aggregate flight observer
   */
  flight_observer_t &get_flight_observer( void)
    {
    return flight_observer;
    }

  INS_type 		ins;
  INS_type 		ins_magnetic;

private:
  atmosphere_t 		atmosphere;
  float 		pitot_pressure;
  float 		TAS;
  float 		IAS;
  float3vector 		GNSS_velocity;
  float			GNSS_speed;
  float3vector 		GNSS_acceleration;
  float 		GNSS_heading;
  float 		GNSS_altitude;
  float3vector 		true_airspeed;
  flight_observer_t 	flight_observer;
  vario_integrator_t 	vario_integrator;
};

#endif /* NAVIGATORT_H_ */
