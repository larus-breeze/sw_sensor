#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "main.h"
#include "uart6.h"
#include "stdio.h"
#include "bt_hm_11.h"

#define ITM_TRACE_ENABLE 1

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


bool Bluetooth_Cmd(uint8_t *cmd)
{
  uint8_t rxByte = 0;
  uint8_t detector = 0;
  uint32_t length = 0;
  while(cmd[length] != 0)
    {
      length++;
    }

  /*Flush UART RX queue*/
  while(true == UART6_Receive(&rxByte))
    {
#if ITM_TRACE_ENABLE
      ITM_SendChar(rxByte);
#endif
    }

  /* Send command*/
  UART6_Transmit(cmd, length);

  uint32_t timeout = 1000;
  while(timeout > 0){
      timeout--;
      delay(1);
      if(true == UART6_Receive(&rxByte))
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
	    }
	  if((rxByte == '+')  && (detector == 2))
	    {
	      /*Bluetooth module acknowledged with OK+.*/
	      return true;
	    }
	}
  }
  //ASSERT(timeout > 0); /*Bluetooth might already be paared this should not be the case here*/
  return false; /*No proper acknowledge from Bluetooth module*/
}

static uint8_t baudratecmd[] = "AT+BAUD4\r\n";
static uint8_t baudratequerry[] = "AT+BAUD?\r\n";
static uint8_t getpowercmd[] = "AT+POWE?\r\n";
bool Bluetooth_Init(void)
{
  UART6_Init();
  // Reset HM 11. Pull low for 100ms to reset
  HAL_GPIO_WritePin(BL_RESETB_GPIO_Port, BL_RESETB_Pin, GPIO_PIN_RESET);
  delay(100);
  HAL_GPIO_WritePin(BL_RESETB_GPIO_Port, BL_RESETB_Pin, GPIO_PIN_SET);
  delay(200);

  //UART6_ChangeBaudRate(9600);

  UART6_ChangeBaudRate(115200);


  return false;
}

void Bluetooth_Transmit(uint8_t *pData, uint16_t Size)
{
  UART6_Transmit(pData, Size);
}

bool Bluetooth_Receive(uint8_t *pRxByte)
{
  UART6_Receive(pRxByte);
}

