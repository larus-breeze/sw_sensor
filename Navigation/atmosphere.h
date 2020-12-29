/*
 * atmosphere.h
 *
 *  Created on: Mar 30, 2015
 *      Author: schaefer
 */

#ifndef APPLICATION_ATMOSPHERE_H_
#define APPLICATION_ATMOSPHERE_H_

#include "vsqrtf.h"

#define RECIP_STD_DENSITY_TIMES_2 1.632f

class atmosphere_t
{
public:
  atmosphere_t( float p_abs)
  : pressure ( p_abs)
  {}
  void set_pressure( float p_abs)
  {
    pressure = p_abs;
  }
  float get_pressure( void) const
  {
    return pressure;
  }
  float get_density( void) const
  {
    return  1.0496346613e-5f * pressure + 0.1671546011f;
  }
  float get_altitude( void) const //!< get NEGATIVE altitude
  {
    float tmp = 8.104381531e-4f * pressure;
    return - tmp * tmp  + 0.20867299170f * pressure - 14421.43945f;
  }
  float get_TAS_from_dynamic_pressure( float dynamic_pressure) const
  {
    return dynamic_pressure < 0.0f ? 0.0f : VSQRTF( 2 * dynamic_pressure / get_density());
  }
  float get_IAS_from_dynamic_pressure( float dynamic_pressure) const
  {
    return dynamic_pressure < 0.0f ? 0.0f : VSQRTF( dynamic_pressure * RECIP_STD_DENSITY_TIMES_2);
  }
private:
  float pressure;
};

#endif /* APPLICATION_ATMOSPHERE_H_ */
