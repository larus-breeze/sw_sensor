/*
 * windobservert.h
 *
 *  Created on: Sep 10, 2021
 *      Author: schaefer
 */

#ifndef WINDOBSERVER_H_
#define WINDOBSERVER_H_

#include "float3vector.h"
#include "AHRS.h"

class wind_observer_t
{
public:
  wind_observer_t (float beta_design)
  : decimating_counter(DECIMATION)
  {
//  beta_max = exp( - T_sample / Tau );
    beta_max = 0.982f; // todo implement correct parameter setup
  }
  const float3vector & get_output( void) const
  {
    return present_output;
  }

  void update (const float3vector &current_value, float3vector heading_vector, circle_state_t state)
  {
    stage_1_N = stage_1_N * beta_max + current_value.e[NORTH] * ( 1.0f - beta_max);
    stage_1_E = stage_1_E * beta_max + current_value.e[EAST]  * ( 1.0f - beta_max);

    --decimating_counter;
    if( decimating_counter == 0)
      {
	decimating_counter = DECIMATION;

	float alpha_N, beta_N, alpha_E, beta_E;

	if( state == STRAIGHT_FLIGHT)
	  {
	    beta_N = beta_E = beta_max;
	    alpha_N = alpha_E = 1.0f - beta_max;
	  }
	else
	  {
	    alpha_N = (1.0f - beta_max) * SQR( heading_vector[NORTH]);
	    alpha_E = (1.0f - beta_max) * SQR( heading_vector[EAST]);
	    beta_N = 1.0f - alpha_N;
	    beta_E = 1.0f - alpha_E;
	  }

	present_output.e[NORTH] = present_output.e[NORTH] * alpha_N + stage_1_N * beta_N;
	present_output.e[EAST]  = present_output.e[EAST]  * alpha_E + stage_1_E * beta_E;
      }

    if( !isnormal( present_output.e[EAST]) || !isnormal( present_output.e[NORTH]))
      present_output.e[NORTH] = present_output.e[EAST] = 0.0f;
  }

 private:
  enum { DECIMATION = 55};
  float stage_1_N;
  float stage_1_E;
  uint32_t decimating_counter;
  float beta_max; // nominator coefficient configured

  float3vector present_output; // maintained to save computing time
};

#endif /* WINDOBSERVERT_H_ */
