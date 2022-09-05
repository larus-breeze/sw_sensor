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

#include "generic_CAN_driver.h"

#ifdef __cplusplus

namespace CAN_driver_ISR // need a namespace to declare friend functions
{
  extern "C" void CAN1_RX0_IRQHandler(void);
  extern "C" void CAN1_TX_IRQHandler(void);
  extern "C" void CAN1_SCE_IRQHandler( void);
}

//! CAN driver module for CAN1 of the STM32F407
class can_driver_t
{
  friend void CAN_driver_ISR::CAN1_RX0_IRQHandler(void);
  friend void CAN_driver_ISR::CAN1_TX_IRQHandler(void);
  friend void CAN_driver_ISR::CAN1_SCE_IRQHandler(void);
public:
  can_driver_t (void);
  void initialize(void);
  inline bool receive( CANpacket &packet, uint32_t wait=INFINITE_WAIT)
  {
	  return  RX_queue.receive( packet, wait);
  }
  bool send( const CANpacket &packet, uint32_t wait=0xffffffff)
  {
    if( locked)
      return true; // silently ignore request, CAN not ready

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
  void reset(void);
private:
  Queue <CANpacket> RX_queue;
  Queue <CANpacket> TX_queue;
  timer reset_timer;
  bool locked;
};

extern COMMON can_driver_t CAN_driver; //!< singleton CAN driver object

void CAN_reset_timer_callback( TimerHandle_t);

#else
QueueHandle_t get_RX_queue( void);
#endif // cplusplus

#endif /* CANDRIVER_H_ */
