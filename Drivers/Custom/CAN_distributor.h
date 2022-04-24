/*
 * CAN_distributor.h
 *
 *  Created on: Feb 16, 2022
 *      Author: schaefer
 */

#ifndef CAN_DISTRIBUTOR_H_
#define CAN_DISTRIBUTOR_H_

#include "candriver.h"

typedef struct
{
  uint16_t ID_mask;
  uint16_t ID_value;
  Queue <CANpacket> * queue;
} CAN_distributor_entry;

bool subscribe_CAN_messages( const CAN_distributor_entry &that);

#endif /* CAN_DISTRIBUTOR_H_ */
