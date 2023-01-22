/*
 * sensor_dump.h
 *
 *  Created on: Jan 22, 2023
 *      Author: schaefer
 */

#ifndef SENSOR_DUMP_H_
#define SENSOR_DUMP_H_

#include "NMEA_format.h"

void decimate_sensor_dump( const output_data_t &output_data);
void format_sensor_dump( const output_data_t &output_data, string_buffer_t &NMEA_buf);

#endif /* SENSOR_DUMP_H_ */
