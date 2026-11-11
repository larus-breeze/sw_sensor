/*
 * sensor_dump.h
 *
 *  Created on: Jan 22, 2023
 *      Author: schaefer
 */

#ifndef SENSOR_DUMP_H_
#define SENSOR_DUMP_H_

#include "NMEA_format.h"

void decimate_sensor_observations( const measurement_data_t &m, const state_vector_t &x);
void format_sensor_dump( const measurement_data_t &m, const D_GNSS_coordinates_t c, const state_vector_t x, string_buffer_t &NMEA_buf);

#endif /* SENSOR_DUMP_H_ */
