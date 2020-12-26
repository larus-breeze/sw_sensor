/** ***********************************************************************
 * @file		KalmanVario_t.cpp
 * @brief		Kalman Filter for vertical navigation (i.e. altitude)
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#include <KalmanVario.h>

ROM float KalmanVario_t::Gain[N][L]= //!< Kalman Gain
    {
	   0.012844708434742,   0.000274837058856,
	   0.016619649826607,   0.004663237320765,
	   0.010104117478788,   0.067518328294903,
	  -0.009829280419933,   0.001390893066503
    };

KalmanVario_t::KalmanVario_t ( float _x, float v, float a)
{
  x[0] = _x;
  x[1] = v;
  x[2] = a;
  x[3] = 0;
}

float KalmanVario_t::update( const float altitude, const float acceleration)
{
  // predict x[] by propagating it through the system model
  float x_est_0 = x[0] + Ta * x[1] + Ta_s_2 * x[2];
  float x_est_1 = x[1] + Ta * x[2];
  float x_est_2 = x[2];
  float x_est_3 = x[3];

  float innovation_x = altitude     - x_est_0;
  float innovation_a = acceleration - x_est_2 - x_est_3;

  // x[] correction
  x[0] = x_est_0 + Gain[0][0] * innovation_x + Gain[0][1] * innovation_a;
  x[1] = x_est_1 + Gain[1][0] * innovation_x + Gain[1][1] * innovation_a;
  x[2] = x_est_2 + Gain[2][0] * innovation_x + Gain[2][1] * innovation_a;
  x[3] = x_est_3 + Gain[3][0] * innovation_x + Gain[3][1] * innovation_a;

  return x[1]; // return velocity
}
