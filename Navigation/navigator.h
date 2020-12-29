#ifndef NAVIGATORT_H_
#define NAVIGATORT_H_

#include "GNSS.h"
#include "INS.h"
#include "differentiator.h"
#include "variointegrator.h"
#include "atmosphere.h"
#include "flight_observer.h"

class navigator_t
{
public:
  navigator_t ()
    :ins (0.01),
     atmosphere (101325.0f)
  {};

  void set_from_euler ( float r, float n, float y)
  {
    ins.set_from_euler(r, n, y);
  }
  eulerangle<float> get_euler( void)
  {
    return ins.get_euler();
  }
  const float3vector &get_ins_acc( void)
  {
    return ins.get_acc();
  }
  const quaternion<float> &get_attitude( void)
  {
    return ins.attitude;
  }
  float get_vario_integrator( void ) const
  {
	  return vario_integrator.get_value();
  }
  float get_TAS( void)
  {
      return TAS;
  }
  float get_IAS( void)
  {
      return IAS;
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

  /**
   * @brief return aggregate flight observer
   */
  flight_observer_t &get_flight_observer( void)
    {
    return flight_observer;
    }

  INS_type 		ins;

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
