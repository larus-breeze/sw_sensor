/*
 * data_structures.h
 *
 *  Created on: Dec 29, 2020
 *      Author: schaefer
 */

#ifndef DATA_STRUCTURES_H_
#define DATA_STRUCTURES_H_

#include "system_configuration.h"
#include "float3vector.h"
#include "euler.h"
#include "quaternion.h"
#include "GNSS.h"

#pragma pack(push, 1)

typedef struct // legacy data type
{
  float3vector acc;
  float3vector gyro;
  float3vector mag;
#if OLD_FORMAT == 0
  float3vector MEMS_gyro;
#endif
  float pitot_pressure;
  float static_pressure;
} measurement_data_t;

typedef struct
{
  measurement_data_t m;
  coordinates_t c;
} input_data_t;

typedef struct
{
  measurement_data_t m;
  coordinates_t c;
  float IAS;
  float TAS;
  float vario_uncompensated;
  float vario;
  float speed_compensation_TAS;
  float speed_compensation_INS;
  float integrator_vario;
  float3vector wind;
  uint32_t circle_mode;
  float3vector nav_acceleration_gnss;
  float3vector nav_induction_gnss;
  float3vector nav_correction;
  float3vector gyro_correction;
  quaternion<float> q;
  eulerangle<float> euler;
  float effective_vertical_acceleration;

  float3vector nav_acceleration_mag;
  float3vector nav_induction_mag;
  eulerangle<float> euler_magnetic;
  quaternion<float> q_magnetic;
} output_data_t;

#pragma pack(pop)

#endif /* DATA_STRUCTURES_H_ */
