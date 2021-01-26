/** ***********************************************************************
 * @file		KalmanVario.h
 * @brief		Kalman Filter for vertical navigation (i.e. altitude)
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef APPLICATION_KALMANVARIO_H_
#define APPLICATION_KALMANVARIO_H_

#include "embedded_memory.h"
#include <stdint.h>
#include "arm_math.h"
#include "system_configuration.h"

class KalmanVario_t
{
private:

  // constants
  enum
  {
    N = 4,  //!< size of state vector x = { altitude, vario, vertical_acceleration }
    L = 2  //!< number of measurement channels = { altitude, vertical_acceleration_measurement }
  };
  static constexpr float Ta = 0.01f; 			//!< sampling rate
  static constexpr float Ta_s_2 = Ta * Ta / 2.0f; 	//!< sampling rate
  static ROM float Gain[N][L];				//!< Pre-computed Kalman Gain

  // variables
  float x[N];	//!< state vector: altitude, vario, acceleration, acceleration offset

public:
  typedef enum// state vector components
  {
    ALTITUDE, VARIO, ACCELERATION_OBSERVED, ACCELERATION_OFFSET
  }  state;
  KalmanVario_t ( float x=0.0, float v=0.0, float a=0.0);

  float update( const float altitude, const float acceleration);

  inline float get_x( state index) const
  {
    if( index <= ACCELERATION_OFFSET)
      return x[index];
    else
      return x[ACCELERATION_OBSERVED] + x[ACCELERATION_OFFSET]; // = acceleration minus offset
  };
};

#endif /* APPLICATION_KALMANVARIO_H_ */
