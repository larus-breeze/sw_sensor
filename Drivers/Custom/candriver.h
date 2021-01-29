/**
 @file candriver.cpp
 @brief CAN bus driver for STM3240G-Eval
 @author: Dr. Klaus Schaefer
 */

#ifndef CANDRIVER_H_
#define CANDRIVER_H_

#include  "system_configuration.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"

//! basic CAN packet type
typedef struct
{
  uint16_t id; 	//!< identifier
  uint16_t dlc; 	//!< data length code
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
} CANpacket;

#ifdef __cplusplus

namespace CAN_driver_ISR // need a namespace to declare friend functions
{
  extern "C" void CAN1_RX0_IRQHandler(void);
  extern "C" void CAN1_TX_IRQHandler(void);
}

//! CAN driver module for CAN1 of the STM32F407
class can_driver_t
{
  friend void CAN_driver_ISR::CAN1_RX0_IRQHandler(void);
  friend void CAN_driver_ISR::CAN1_TX_IRQHandler(void);
public:
  can_driver_t (void);

  inline bool receive( CANpacket &packet, uint32_t wait=INFINITE_WAIT)
  {
	  return  RX_queue.receive( packet, wait);
  }
  bool send( const CANpacket &packet, uint32_t wait=0xffffffff)
  {
    /* Temporarily disable Transmit mailbox empty Interrupt */
    CAN1->IER &= ~CAN_IT_TX_MAILBOX_EMPTY;
    if ( send_can_packet( packet) == true) // hardware FIFO
      {
      /* Enable Transmit mailbox empty Interrupt */
      CAN1->IER |= CAN_IT_TX_MAILBOX_EMPTY;
      return true;
      }

    bool ret = TX_queue.send( packet, wait);

    /* Enable Transmit mailbox empty Interrupt */
    CAN1->IER |= CAN_IT_TX_MAILBOX_EMPTY;

    return ret;
  }
  bool send_can_packet( const CANpacket &msg); //!< helper function
  Queue <CANpacket> get_RX_Queue( void ) const
		{
	  	  return RX_queue;
		}
private:
  Queue <CANpacket> RX_queue;
  Queue <CANpacket> TX_queue;
};

extern COMMON can_driver_t CAN_driver; //!< singleton CAN driver object

#else
QueueHandle_t get_RX_queue( void);
#endif // cplusplus
#endif /* CANDRIVER_H_ */
