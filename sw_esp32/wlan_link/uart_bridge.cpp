/** *****************************************************************************
 * @file    uart_bridge.cpp
 * @brief   3x UART <-> WiFi TCP bridge
 *
 * Bluetooth Classic support (BluetoothSerial) is temporarily removed - see
 * the doc comment in uart_bridge.h. Assumes WiFi (access point and/or
 * station, see wifi_config.h) has already been brought up by
 * setupWifi() before setupUartBridge() runs.
 *
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#include "uart_bridge.h"
#include <WiFi.h>

#define NUM_COM   3

// COM[0]/Serial is also the USB/programmer debug console (see the doc
// comment below `COM[0]->write` further down) - 115200, not 38400 like
// the other two real UART<->WiFi bridges, to match a typical serial
// monitor's default. wlan_link.ino's own Serial.begin() at boot must be
// kept in sync with this value - see its doc comment.
#define UART_BAUD0      115200
#define SERIAL_PARAM0   SERIAL_8N1
#define SERIAL0_RXPIN   21
#define SERIAL0_TXPIN    1
#define SERIAL0_TCP_PORT 8882

#define UART_BAUD1      38400
#define SERIAL_PARAM1   SERIAL_8N1
#define SERIAL1_RXPIN   16
#define SERIAL1_TXPIN   17
#define SERIAL1_TCP_PORT 8881

#define UART_BAUD2      38400
#define SERIAL_PARAM2   SERIAL_8N1
#define SERIAL2_RXPIN   15
#define SERIAL2_TXPIN    4
#define SERIAL2_TCP_PORT 8880

#define BUFFER_SIZE      1024
#define MAX_NMEA_CLIENTS    4

static HardwareSerial Serial_one (1);
static HardwareSerial Serial_two (2);
static HardwareSerial *COM[NUM_COM] = { &Serial, &Serial_one, &Serial_two };

static WiFiServer server_0 (SERIAL0_TCP_PORT);
static WiFiServer server_1 (SERIAL1_TCP_PORT);
static WiFiServer server_2 (SERIAL2_TCP_PORT);
static WiFiServer *tcpServer[NUM_COM] = { &server_0, &server_1, &server_2 };
static WiFiClient tcpClient[NUM_COM][MAX_NMEA_CLIENTS];

static uint8_t uartToNetBuf[NUM_COM][BUFFER_SIZE];
static uint16_t uartToNetLen[NUM_COM];

static uint8_t netToUartBuf[BUFFER_SIZE];

void setupUartBridge (void)
{
  COM[0]->begin (UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
  COM[1]->begin (UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  COM[2]->begin (UART_BAUD2, SERIAL_PARAM2, SERIAL2_RXPIN, SERIAL2_TXPIN);

  COM[0]->println ("Starting TCP Server 1");
  tcpServer[0]->begin ();
  tcpServer[0]->setNoDelay (true);
  COM[1]->println ("Starting TCP Server 2");
  tcpServer[1]->begin ();
  tcpServer[1]->setNoDelay (true);
  tcpServer[2]->begin ();
  tcpServer[2]->setNoDelay (true);
}

void handleUartBridge (void)
{
  // accept new TCP clients, replacing any disconnected slot
  for (int num = 0; num < NUM_COM; ++num)
    {
      if (tcpServer[num]->hasClient ())
        {
          bool placed = false;
          for (int slot = 0; slot < MAX_NMEA_CLIENTS; ++slot)
            {
              if ((! tcpClient[num][slot]) || (! tcpClient[num][slot].connected ()))
                {
                  if (tcpClient[num][slot])
                    tcpClient[num][slot].stop ();
                  tcpClient[num][slot] = tcpServer[num]->accept ();
                  placed = true;
                  break;
                }
            }
          if (! placed) // no free slot - reject
            {
              WiFiClient rejected = tcpServer[num]->accept ();
              rejected.stop ();
            }
        }
    }

  for (int num = 0; num < NUM_COM; ++num)
    {
      // TCP -> UART
      for (int slot = 0; slot < MAX_NMEA_CLIENTS; ++slot)
        {
          if (tcpClient[num][slot])
            {
              uint16_t n = 0;
              while (tcpClient[num][slot].available () && (n < BUFFER_SIZE - 1))
                netToUartBuf[n++] = tcpClient[num][slot].read ();
              COM[num]->write (netToUartBuf, n);
            }
        }

      // UART -> TCP + RS232 echo
      if (COM[num]->available ())
        {
          uartToNetLen[num] = 0;
          // bounded the same way as the TCP -> UART loop above - a
          // continuously-streaming UART (now that the STM32 side actually
          // runs instead of crash-looping) could otherwise keep this loop
          // spinning for as long as available() stays true, starving
          // loop() of the handleWebServerClient() calls the web UI needs
          while (COM[num]->available () && (uartToNetLen[num] < BUFFER_SIZE - 1))
            {
              uartToNetBuf[num][uartToNetLen[num]] = COM[num]->read ();
              ++uartToNetLen[num];
            }

          for (int slot = 0; slot < MAX_NMEA_CLIENTS; ++slot)
            if (tcpClient[num][slot])
              tcpClient[num][slot].write (uartToNetBuf[num], uartToNetLen[num]);

          // Deliberately NOT echoed to COM[0] (the USB/programmer serial
          // debug console) - that would mix live NMEA/telemetry traffic
          // into the debug log output. The TCP path above already carries
          // this data to the network; COM[1] (RS232) still gets it since
          // that's a real hardware output some external device may depend
          // on, not a debug console.
          COM[1]->write (uartToNetBuf[num], uartToNetLen[num]); // echo to RS232 output
        }
    }
}
