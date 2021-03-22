#include <bt_hm.h>
#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "main.h"
#include "uart6.h"
#include "stdio.h"

#define ITM_TRACE_ENABLE 0

#if ACTIVATE_BLUETOOTH_TEST

static char altitude[]  = "$PGRMZ,246,f,3*1B\r\n";

void bluetooth_hm_11(void*)
{
  acquire_privileges();
  UART6_Init();

  // Reset HM 11. Pull low for 100ms to reset
  HAL_GPIO_WritePin(BL_RESETB_GPIO_Port, BL_RESETB_Pin, GPIO_PIN_RESET);
  delay(100);
  HAL_GPIO_WritePin(BL_RESETB_GPIO_Port, BL_RESETB_Pin, GPIO_PIN_SET);
  delay(200);

  uint8_t len  = strlen(altitude);
  uint8_t rxData = 0;
  for(;;)
    {
      UART6_Transmit((uint8_t *)altitude, len);

      delay(1000);

      while(UART6_Receive(&rxData) == true)
	{

	}
    }
}

RestrictedTask bluetooth_handling(bluetooth_hm_11, "BT_HM_11", 256);

#endif


#define BLUETOOTH_DEFAULT_UART_RX_TIMEOUT 500
#define BLUETOOTH_CONNECTION_TIMEOUT 5000
static bool ble_connected = false;

void Bluetooth_SendCmd(const uint8_t *cmd)
{
  uint32_t length = 0;
  while(cmd[length] != 0)
    {
      length++;
    }
  /* Send command*/
  UART6_Transmit(cmd, length);
#if ITM_TRACE_ENABLE
  ITM_SendChar('\r');
  ITM_SendChar('\n');
  for(uint32_t i = 0; i < length; i++)
    {
      ITM_SendChar(cmd[i]);
    }
#endif
}


void Bluetooth_FlushRx(void)
{
  uint8_t rxByte = 0;
  while(true == UART6_Receive(&rxByte, 0));
}


bool Bluetooth_Cmd(const uint8_t *cmd)
{
  uint8_t rxByte = 0;
  uint8_t detector = 0;

  /*Give module some time to finish previous command*/
  delay(50);

  /*Flush UART RX queue*/
  Bluetooth_FlushRx();

  /* Send command*/
  Bluetooth_SendCmd(cmd);

#if ITM_TRACE_ENABLE
  ITM_SendChar('\r');
  ITM_SendChar('\n');
#endif

  while(true)
    {
      if(true == UART6_Receive(&rxByte, BLUETOOTH_DEFAULT_UART_RX_TIMEOUT))
	{
#if ITM_TRACE_ENABLE
	  ITM_SendChar(rxByte);
#endif
	  if((rxByte == 'O')  && (detector == 0))
	    {
	      detector = 1;
	    }
	  if((rxByte == 'K')  && (detector == 1))
	    {
	      detector = 2;
	      while(true == UART6_Receive(&rxByte, 1));  /*Get bytes after OK*/
	      return true; /*Bluetooth module acknowledged with OK */
	    }
	}
      else
	{
	  break;
	}

    }
  return false; /*No proper acknowledge from Bluetooth module*/
}

/*   It seems that it is required  to recognize a lost connection! In that case we need to stop sending
 *   data until connection is established again!  Reported by OK+CONN (HM-19)
 *   Tested HM-19 with SONY XA2 Ultra LineagOS.  Bluetooth stopped working after a while. Only a reboot helped.
 *   Works acceptable with Samsung XCover Pro,   Sony XA2 Ultra,   CAT S52.  Deactivating and activating
 *   connection from XCSOAR works good. Even if lots of data is continuously transferred.
 *
 *
 * The different Modules:
 *
 * HM-11 ordered in Germany
 * AT+VERS?	HMSoft V545
 * AT+IMME1
 * AT+BAUD4 	(115200)
 * AT+NAMEHM11DE
 * AT+RESET
 * AT+START
 * Does not report a lost connection!
 *
 * HM-11 ordered in China
 * AT+VERS?	HMSoft V610
 * AT+BAUD4  	(115200)
 * AT+POWE3   	Set to 6dm
 * Reports connection and lost.
 * Locked me out!  Module only talked again via UART after a power cycle. (Caused by pressing reconnect from XCSOAR)
 *
 * HM-19 ordered in China
 * AT+VERS?	HMSoft V114 (could not
 * HM-19 commands			Answer
 * AT        						OK  |     OK+LOST\r\n
 * AT+RESET						OK+RESET
 * AT+VERS?						OK+Get:HMSoft V114
 * AT+NAME?						OK+GET:HMSoft
 * AT+START						OK+CONN\r\n
 * AT+BAUD7 (115200)  Effect only after reset. 		OK+SET:7
 *
 * Configuration commands:
 */

/* \r\n not required. "AT Command are fixed length commands and new line is this redundant. "HowToUse Hm-1x.pdf
 * This can not be true especially for setting a custom NAME?*/
ROM uint8_t baudratecmd[] = "AT+BAUD7";  /* Change to 115200 baud  HM.19*/
ROM uint8_t setName[] = BLUETOOTH_NAME;
ROM uint8_t setPIN[] = "AT+PIN000000";
ROM uint8_t interruptModule[] = "AT";
ROM uint8_t resetModule[] = "AT+RESET";
//static uint8_t getpowercmd[] = "AT+POWE?\r\n";
//static uint8_t desirecmode[] = "AT+MODE0\r\n";   //AT- configure prior connection, transparent uart after connection
//static uint8_t disableConnecting[] = "AT+IMME1";  /* Disable automatic connection*/
//static uint8_t enableConnecting[] = "AT+IMME0";  /* Enable automatic connection*/
//static uint8_t resetToFactorySettings[] = "AT+RENEW";
//static uint8_t getName[] = "AT+NAME?";

void Bluetooth_Reset(void)
{
  // Reset HM 11. Pull low for 100ms to reset
  HAL_GPIO_WritePin(BL_RESETB_GPIO_Port, BL_RESETB_Pin, GPIO_PIN_RESET);
  delay(200);
  HAL_GPIO_WritePin(BL_RESETB_GPIO_Port, BL_RESETB_Pin, GPIO_PIN_SET);
  delay(50);
}
#define BT_CONFIGURE 1

bool Bluetooth_Init(void)
{
  Bluetooth_Reset(); /* Its in reset mode prior to this.*/

  UART6_DeInit(); /* Stop driving TX line.*/
  UART6_Init();  /* Stop here and connect external UART adapter for debugging.*/

#if BT_CONFIGURE
  /*First try to recognize used baudrate*/
  delay(500); /*Let module wake up after reset*/
  UART6_ChangeBaudRate(9600);
  if(true == Bluetooth_Cmd(interruptModule))
    {
      /*Modules uses Baudrate 9600, and thus has never configured before!*/
      Bluetooth_Cmd(setName);
      Bluetooth_Cmd(setPIN);
      Bluetooth_Cmd(baudratecmd);
      Bluetooth_Cmd(resetModule);
    }
  delay(500);
#endif

  UART6_ChangeBaudRate(115200);
  if(true == Bluetooth_Cmd(interruptModule))
    {
      /*Seems that bluetooth modules is configured and answers at 115200 baud.*/
      update_system_state_set(BLUEZ_OUTPUT_ACTIVE);
    }

  delay(500); /*Delay after last AT command.*/
  return true;
}


void Bluetooth_Transmit(uint8_t *pData, uint16_t Size)
{
  if(true == ble_connected)
    {
      /* Only transmit if module is connected.*/
      UART6_Transmit(pData, Size);
    }
}

bool Bluetooth_Receive(uint8_t *pRxByte, uint32_t timeout)
{
  return UART6_Receive(pRxByte, timeout);
}

static void BLE_runnable (void*)
{
  Bluetooth_Init();
  delay(500);

  uint8_t rxByte = 0;
  uint32_t detector = 0;
  for(;;)
    {
      if(true == Bluetooth_Receive(&rxByte, BLUETOOTH_CONNECTION_TIMEOUT))
	{
	  /*Detect and parse response messages from BLE module*/
	  if('+' == rxByte)
	    {
	      /*Restart detector on '+' reception*/
	      detector = 1;
	    }

	  if(('C' == rxByte) && (detector == 1))
	    {
	      detector = 2;
	    }
	  if(('O' == rxByte) && (detector == 2))
	    {
	      detector = 3;
	    }
	  if(('N' == rxByte) && (detector == 3))
	    {
	      detector = 4;
	    }
	  if(('N' == rxByte) && (detector == 4))
	    {
	      detector = 0;
	      /*Detected OK+CONN*/
	      ble_connected = true;
	    }


	  if(('L' == rxByte) && (detector == 1))
	    {
	      detector = 2;
	    }
	  if(('O' == rxByte) && (detector == 2))
	    {
	      detector = 3;
	    }
	  if(('S' == rxByte) && (detector == 3))
	    {
	      detector = 4;
	    }
	  if(('T' == rxByte) && (detector == 4))
	    {
	      detector = 0;
	      /*Detected OK+LOST.  Do a period reset here as sometimes reconnection is not possible*/
	      ble_connected = false;

	    }
	}
      else
	{
	  /*Nothing received. Do a reset after a 5 seconds if not connected.*/
	  if(false == ble_connected)
	  {
	      Bluetooth_Reset();
	  }
	}
    }
}

Task bluetooth_task (BLE_runnable, "BLE", 256, 0, BLUETOOTH_PRIORITY);
