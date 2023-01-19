#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "common.h"
#include "NMEA_format.h"

#include <bt_hm.h>
#include "NMEA_format.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usart_1_driver.h"
#include "usart_2_driver.h"
#include "communicator.h"
#include "system_state.h"

COMMON string_buffer_t NMEA_buf;
extern USBD_HandleTypeDef hUsbDeviceFS; // from usb_device.c

static void runnable (void* data)
{

  #if ACTIVATE_USB_NMEA
  MX_USB_DEVICE_Init();
  update_system_state_set( USB_OUTPUT_ACTIVE);
#endif

#if ACTIVATE_USART_2_NMEA
  USART_2_Init ();
  update_system_state_set( USART_2_OUTPUT_ACTIVE);
#endif
#if ACTIVATE_USART_1_NMEA
  USART_1_Init ();
  update_system_state_set( USART_1_OUTPUT_ACTIVE);
#endif

//  suspend(); // wait until we are needed

  for (synchronous_timer t (NMEA_REPORTING_PERIOD); true; t.sync ())
    {

      format_NMEA_string( output_data, NMEA_buf, 0); // todo fixme declination set to 0 for a while

#if ACTIVATE_USB_NMEA
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)NMEA_buf.string, NMEA_buf.length);
      USBD_CDC_TransmitPacket(&hUsbDeviceFS);
#endif
#if ACTIVATE_BLUETOOTH_NMEA
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

ROM float declination_DUMMY = 2.0f; // todo fixme !

COMMON RestrictedTask NMEA_task( runnable, "NMEA", 256, 0, (NMEA_USB_PRIORITY) | portPRIVILEGE_BIT);

