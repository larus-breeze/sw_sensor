/*
 * data_structures.h
 *
 *  Created on: Dec 29, 2020
 *      Author: schaefer
 */

#ifndef DATA_STRUCTURES_H_
#define DATA_STRUCTURES_H_

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
  float3vector MEMS_gyro;
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
  eulerangle<float> euler;
  float3vector nav_acceleration_ins;
  float3vector nav_acceleration_gnss;
  quaternion<float> q;
  float3vector nav_correction;
  float3vector gyro_correction;
  float effective_vertical_acceleration;
  uint32_t circle_mode;
} output_data_t;

#pragma pack(pop)

#endif /* DATA_STRUCTURES_H_ */
