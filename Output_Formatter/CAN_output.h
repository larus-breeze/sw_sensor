/** ***********************************************************************
 * @file		CAN_output.h
 * @brief		format internal data and send to CAN
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#ifndef SRC_CAN_OUTPUT_H_
#define SRC_CAN_OUTPUT_H_

#include "system_configuration.h"

#include "navigator.h"
#include "flight_observer.h"
#include "NMEA_format.h"
#ifdef UNIX
#include "USB_serial.h"
#include "stdint.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include "stdio.h"
#include "sstream"
#include "stdlib.h"
#include "string.h"

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

#pragma pack(push, 2)

class CAN_gateway_packet
{
public:
  CAN_gateway_packet( const CANpacket &p)
  : ID_checked(p.id),
    DLC_checksum(p.dlc),
    data(p.data_l)
  {
    uint16_t checksum = ~(p.id % 31);
    ID_checked |= checksum << 11;

    checksum = ~(p.data_l % 4095);
    DLC_checksum |= checksum << 4;
  }

  bool to_CANpacket( CANpacket &p)
  {
    uint16_t checksum = ~((ID_checked & 0x7ff) % 31) & 0x1f;
    if( ID_checked >> 11 != checksum)
      return false;

    checksum = ~(data % 4095) & 0xfff;
    if( DLC_checksum >> 4 != checksum)
      return false;

    p.id     = ID_checked & 0x7ff;
    p.dlc    = DLC_checksum & 0x0f;
    p.data_l = data;

    return true;
  }

public:
  uint16_t ID_checked;
  uint16_t DLC_checksum;
  uint64_t data; //
};

#pragma pack(pop)

class CAN_driver_t
{
public:
  bool send( CANpacket p, int dummy)
  {
    CAN_gateway_packet output( p);
    write_usb_serial( (uint8_t *) &output, sizeof output);
    return true;
  }
};

extern CAN_driver_t CAN_driver;
void CAN_output ( const output_data_t &x);

#else
#include "candriver.h"
extern RestrictedTask CAN_task;

inline void trigger_CAN(void)
{
  CAN_task.notify_give();
}

#endif

#endif /* SRC_CAN_OUTPUT_H_ */
