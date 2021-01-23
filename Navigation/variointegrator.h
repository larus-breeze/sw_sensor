/*
 * variointegrator.h
 *
 *  Created on: Sep 5, 2020
 *      Author: schaefer
 */

#ifndef VARIOINTEGRATOR_H_
#define VARIOINTEGRATOR_H_

#include "INS.h"
#define SQR(x) ((x)*(x))
#define SIN(x) sinf(x)
#include "pt2.h"

class vario_integrator_t
{
public:
  vario_integrator_t(void);
  float get_value(void) const;
  void update(float vario, float heading, circle_state_t circling);
private:
  enum{ N_SECTORS=18};
  float sector_averages[N_SECTORS];
  uint32_t used_sectors;
  pt2<float,float> integrator_averager;
  circle_state_t present_state;
};

#endif /* VARIOINTEGRATOR_H_ */
