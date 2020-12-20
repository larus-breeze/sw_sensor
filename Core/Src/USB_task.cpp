/** *****************************************************************************
 * @file    USB_task.cpp
 * @author  Klaus Schaefer
 * @brief   USB CDC functionality
 ******************************************************************************/

#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "memory.h"
#include "usb_device.h"
#include "usbd_cdc.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

ROM uint8_t text[]="The quick brown fox jumps over the lazy dogs head 01234567890 \r\n";

void USB_runnable( void *)
{
	MX_USB_DEVICE_Init();

	for( synchronous_timer t(100); true; t.sync())
	{
		USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)text, 64);
		USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	}
}

RestrictedTask USB_task(USB_runnable, "USB", 256, 0, STANDARD_TASK_PRIORITY | portPRIVILEGE_BIT);
