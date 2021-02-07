#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "common.h"
#include "NMEA_Output.h"
#include "NMEA_format.h"
#include "usb_device.h"
#include "usbd_cdc.h"

#if ACTIVATE_USB_NMEA

COMMON NMEA_buffer_t NMEA_buf;
extern USBD_HandleTypeDef hUsbDeviceFS; // from usb_device.c

static void runnable (void*)
{
  MX_USB_DEVICE_Init();

  for (synchronous_timer t (NMEA_REPORTING_PERIOD); true; t.sync ())
    {
      char *next;

      format_RMC (GNSS, NMEA_buf.string);
      next = NMEA_append_tail (NMEA_buf.string);

      format_GGA (GNSS, next);
      next = NMEA_append_tail (next);

      format_MWV (output_data.wind[NORTH], output_data.wind[EAST], next);
      next = NMEA_append_tail (next);

      format_PTAS1 (output_data.vario,
		    output_data.integrator_vario,
		    output_data.c.position.e[DOWN] * -1.0,
		    output_data.TAS,
		    next);
      next = NMEA_append_tail (next);

      NMEA_buf.length = next - (const char*) (NMEA_buf.string);

      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)NMEA_buf.string, NMEA_buf.length);
      USBD_CDC_TransmitPacket(&hUsbDeviceFS);

    }
}

RestrictedTask NMEA_task( runnable, "NMEA", 256, 0, STANDARD_TASK_PRIORITY | portPRIVILEGE_BIT);

#endif
