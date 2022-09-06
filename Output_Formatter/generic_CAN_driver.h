/*
 * generic_CAN_driver.h
 *
 *  Created on: Sep 5, 2022
 *      Author: schaefer
 */

#ifndef GENERIC_CAN_DRIVER_H_
#define GENERIC_CAN_DRIVER_H_

#include "stdint.h"

//! basic CAN packet type
class CANpacket
{
public:
  CANpacket( uint16_t _id=0, uint16_t _dlc=0, uint64_t _data=0)
  : id(_id),
    dlc(_dlc),
    data_l(_data)
  {}
  bool operator ==(const CANpacket&right)
    {
      return 	(id == right.id) &&
		(dlc = right.dlc) &&
		(data_l == right.data_l);
    }
  uint16_t id; 	//!< identifier
  uint16_t dlc; //!< data length code
  union
  {
		uint8_t  data_b[8]; 	//!< data seen as 8 times uint8_t
		int8_t   data_sb[8]; 	//!< data seen as 8 times int8_t
		uint16_t data_h[4]; 	//!< data seen as 4 times uint16_t
		int16_t  data_sh[4]; 	//!< data seen as 4 times int16_t
		uint32_t data_w[2]; 	//!< data seen as 2 times uint32_t
		int32_t  data_sw[2];	//!< data seen as 2 times int32_t
		float    data_f[2]; 	//!< data seen as 2 times 32-bit floats
		uint64_t data_l;    	//!< data seen as 64-bit integer
  };
} ;

bool CAN_send( const CANpacket &p, uint32_t max_delay);

#endif /* GENERIC_CAN_DRIVER_H_ */
