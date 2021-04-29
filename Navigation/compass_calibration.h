/*
 * compass_calibration.h
 *
 *  Created on: Apr 27, 2021
 *      Author: schaefer
 */

#ifndef COMPASS_CALIBRATION_H_
#define COMPASS_CALIBRATION_H_

#include "system_configuration.h"
#include "Linear_Least_Square_Fit.h"

class calibration_t
{
public:
  calibration_t( float _offset=0.0f, float slope=1.0f)
  : offset( _offset),
    scale( 1.0f / slope),
    variance ( 1.0e10f)
  {}

  void refresh ( float _offset, float slope, float _variance)
    {
      if( variance > 1.0e9f) // first calibration coming in
	{
	  offset = _offset;
	  scale = 1.0f / slope;
	  variance = _variance;
	}
      else
	{
	  offset   = offset   * MAG_CALIB_LETHARGY + ( _offset        * (1.0f - MAG_CALIB_LETHARGY));
	  scale    = scale    * MAG_CALIB_LETHARGY + ( (1.0f / slope) * (1.0f - MAG_CALIB_LETHARGY)) ;
	  variance = variance * MAG_CALIB_LETHARGY + ( _variance      * (1.0f - MAG_CALIB_LETHARGY));
	}
    }

  float calibrate( float sensor_reading)
  {
    return (sensor_reading - offset) * scale;
  }

  float
  getVariance () const
  {
    return variance;
  }

  void
  setVariance (float variance)
  {
    this->variance = variance;
  }

private:
  float offset; // in sensor units
  float scale; // convert sensor-units into SI-data
  float variance; // measure of precision
};

class compass_calibration_t
{
public:
  compass_calibration_t( void)
    :calibration_done(false)
  {}

  float3vector calibrate( const float3vector &in)
  {
    float3vector out;

    // shortcut while no data available
    if( !calibration_done)
      return in;

    for( unsigned i=0; i<3; ++i)
      out.e[i]=calibration[i].calibrate( in.e[i]);
    return out;
  }
  void set_calibration( linear_least_square_fit<float> mag_calibrator[3])
  {
    if( mag_calibrator[0].get_count() < MINIMUM_MAG_CALIBRATION_SAMPLES)
      return; // not enough entropy

    float new_calibration[3][4];

    for (unsigned i = 0; i < 3; ++i)
      {
        mag_calibrator[i].evaluate (new_calibration[i][0], new_calibration[i][1],
  				  new_calibration[i][2], new_calibration[i][3]);

        calibration[i].refresh (
  	  new_calibration[i][0], new_calibration[i][1],
  	  new_calibration[i][2] + new_calibration[i][3]);
      }

    calibration_done = true; // at least one calibration done now
  }

private:
  bool calibration_done;
  calibration_t calibration[3];
};

#endif /* COMPASS_CALIBRATION_H_ */
