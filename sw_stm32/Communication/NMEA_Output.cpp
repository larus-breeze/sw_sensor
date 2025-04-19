/**
 * @file 	NMEA_Output.cpp
 * @brief 	Task for NMEA output to several serial interfaces
 * @author: 	Klaus Schaefer
 * @copyright 	Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/

#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "common.h"
#include "NMEA_format.h"
#include "CAN_listener.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usart_1_driver.h"
#include "usart_2_driver.h"
#include "bluetooth_HM19.h"
#include "communicator.h"
#include "system_state.h"
#include "sensor_dump.h"
#include "uSD_handler.h"

COMMON string_buffer_t __ALIGNED( sizeof(string_buffer_t)) NMEA_buf;
extern USBD_HandleTypeDef hUsbDeviceFS; // from usb_device.c

static void NMEA_runnable (void* data)
{
  delay( NMEA_START_DELAY);

  bool horizon_available = configuration( HORIZON);

  #if ACTIVATE_USB_NMEA
  MX_USB_DEVICE_Init();
  update_system_state_set( USB_OUTPUT_ACTIVE);
  delay( 1);
#endif

#if ACTIVATE_USART_2_NMEA
  USART_2_Init ();
  update_system_state_set( USART_2_OUTPUT_ACTIVE);
  delay( 1);
#endif
#if ACTIVATE_USART_1_NMEA
  USART_1_Init ();
  update_system_state_set( USART_1_OUTPUT_ACTIVE);
  delay( 1);
#endif

  if( dump_sensor_readings)
  {
    MX_USB_DEVICE_Init(); // force using the USB ACM device
    unsigned i=0;
    for ( synchronous_timer t (10); true; t.sync ())
      {
        decimate_sensor_observations( output_data);
        ++i;
        if( i >=50) // => 2 Hz output rate
  	{
  	  i=0;
  	  format_sensor_dump( output_data, NMEA_buf);
  	  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)NMEA_buf.string, NMEA_buf.length);
  	  USBD_CDC_TransmitPacket(&hUsbDeviceFS);

#if ACTIVATE_USART_1_NMEA
        USART_1_transmit_DMA( (uint8_t *)(NMEA_buf.string), NMEA_buf.length);
#endif
#if ACTIVATE_USART_2_NMEA
        USART_2_transmit_DMA( (uint8_t *)(NMEA_buf.string), NMEA_buf.length);
#endif
  	}
      }
  }

  unsigned decimating_counter = NMEA_DECIMATION_RATIO;
  for (synchronous_timer t (NMEA_REPORTING_PERIOD); true; t.sync ())
    {
      NMEA_buf.length = 0; // start at the beginning of the buffer
      format_NMEA_string_fast( output_data, NMEA_buf, horizon_available);
#if NMEA_DECIMATION_RATIO == 0
      format_NMEA_string_slow( output_data, NMEA_buf);
#else
      if( --decimating_counter == 0)
	{
	  decimating_counter = NMEA_DECIMATION_RATIO;
	  format_NMEA_string_slow( output_data, NMEA_buf);
	}
#endif
      //Check if there is a CAN Message received which needs to be replayed via a Larus NMEA PLARS Sentence.
      float32_t value;
      char *next = NMEA_buf.string + NMEA_buf.length;
      if (get_mc_updates(value))
      {
	  format_PLARS(value, MC, next);
      }
      if (get_bal_updates(value))
      {
	  format_PLARS(value, BAL, next);
      }
      if (get_bugs_updates(value))
      {
	  format_PLARS(value, BUGS, next);
      }
      if (get_qnh_updates(value))
      {
	  format_PLARS(value, QNH, next);
      }
      NMEA_buf.length = next - NMEA_buf.string;



#if ACTIVATE_USB_NMEA
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)NMEA_buf.string, NMEA_buf.length);
      USBD_CDC_TransmitPacket(&hUsbDeviceFS);
#endif
#if ACTIVATE_BLUETOOTH_HM19
      Bluetooth_Transmit( (uint8_t *)(NMEA_buf.string), NMEA_buf.length);
#endif
#if ACTIVATE_USART_1_NMEA
      USART_1_transmit_DMA( (uint8_t *)(NMEA_buf.string), NMEA_buf.length);
#endif
#if ACTIVATE_USART_2_NMEA
      USART_2_transmit_DMA( (uint8_t *)(NMEA_buf.string), NMEA_buf.length);
#endif
    }
}

static TaskParameters_t p =
  {
      NMEA_runnable,
      "NMEA",
      256, 0,
      (NMEA_USB_PRIORITY) | portPRIVILEGE_BIT,
      0,
      {
	{ COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
	{ (void *)&NMEA_buf, sizeof(NMEA_buf), portMPU_REGION_READ_WRITE },
	{ 0, 0, 0 }
      }
   };

COMMON RestrictedTask NMEA_task( p);

