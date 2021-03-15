/** ***********************************************************************
 * @file		variointegrator.cpp
 * @brief		maintain vario average information
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#include <variointegrator.h>
#include "math.h"
#include "my_assert.h"

#define AVG_VARIO_F_BY_FS ( 0.5 / 30.0 / 10.0) // assuming 10 Hz update

vario_integrator_t::vario_integrator_t (void)
  : integrator_averager( AVG_VARIO_F_BY_FS),
    present_state( STRAIGHT_FLIGHT),
    used_sectors(0)
{
  integrator_averager.settle(0.0f);
}

void vario_integrator_t::update (float vario, float heading, circle_state_t state)
{
  if( heading < 0.0f)
    heading += 2.0f * M_PI;

  switch( present_state)
  {
    case STRAIGHT_FLIGHT:
      if( state == CIRCLING)
	{
	  used_sectors=0;
	  present_state = CIRCLING;
	}
      else
	  integrator_averager.respond( vario);
    break;
    case CIRCLING:
      if( state == STRAIGHT_FLIGHT)
	{
	  integrator_averager.settle(vario);
	  present_state = STRAIGHT_FLIGHT;
	}
      else
	{
	  unsigned index = heading / 2.0 / M_PI * N_SECTORS;
	  ASSERT( index < N_SECTORS);
	  used_sectors |= (1 << index);
	  sector_averages[ index]=vario;
	}
     break;
     default:
     break;
  }
}

float vario_integrator_t::get_value (void) const
{
  if( present_state == STRAIGHT_FLIGHT)
      return integrator_averager.get_output();
  else
      {
      unsigned count=0;
      float sum=0.0f;
      for( unsigned i=0; i<N_SECTORS; ++i)
	if( (used_sectors & ( 1 << i)) != 0 )
	  {
	  sum += sector_averages[ i];
	  ++count;
	  }

      if( count == 0)
	return 0.0f;

      sum /= count;
      ASSERT( sum != NAN);
      return sum;
      }
}
