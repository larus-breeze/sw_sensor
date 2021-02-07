/*
 * NMEA_Output.h
 *
 *  Created on: Feb 7, 2021
 *      Author: schaefer
 */

#ifndef SRC_NMEA_OUTPUT_H_
#define SRC_NMEA_OUTPUT_H_

class NMEA_buffer_t
{
public:
  char string[255];
  uint8_t length;
};

extern NMEA_buffer_t NMEA_buf;

#endif /* SRC_NMEA_OUTPUT_H_ */
