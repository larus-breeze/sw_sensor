/** *****************************************************************************
 * @file    uart_bridge.cpp
 * @brief   3x UART <-> WiFi TCP / Bluetooth Classic bridge
 *
 * See the doc comment in uart_bridge.h. Assumes WiFi (access point and/or
 * station, see wifi_config.h) has already been brought up by
 * setupWifi() before setupUartBridge() runs.
 *
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#include "uart_bridge.h"
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <freertos/ringbuf.h>

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

// Bounds how long a write()/read() to one NMEA client can block this
// function for - handleUartBridge() runs on the same single-threaded
// loop() as handleWebServerClient(), so a client that stops ACKing
// (e.g. an Android phone's WiFi power-save briefly pausing delivery)
// would otherwise stall the web server and the accept() loop below for
// every other client/port too, not just this one.
#define NMEA_CLIENT_TIMEOUT_MS  400

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

//!< retry interval after a failed SerialBT.begin()
#define BT_START_RETRY_MS     30000u
//!< UART -> Bluetooth buffer, drained by btTxTask
#define BT_TX_RINGBUF_SIZE     4096
#define BT_TX_TASK_STACK_SIZE  3072

static BluetoothSerial SerialBT;
static bool btRequested = false;    //!< startBluetoothBridge() was called
static volatile bool btRunning = false; //!< SerialBT.begin() succeeded
static unsigned long btLastStartAttemptMs = 0;
static char btDeviceName[33];

// Static, so creating them can't fail at runtime (heap exhaustion or
// fragmentation) - only SerialBT.begin() needs a retry.
static uint8_t btTxRingbufStorage[BT_TX_RINGBUF_SIZE];
static StaticRingbuffer_t btTxRingbufStruct;
static RingbufHandle_t btTxRingbuf = nullptr;
static StackType_t btTxTaskStack[BT_TX_TASK_STACK_SIZE];
static StaticTask_t btTxTaskTcb;

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

//!< SerialBT.write() blocks up to ~1s per packet while a peer doesn't drain
//!< its data - done here instead of in loop(), so a stalled peer only
//!< drops Bluetooth data and never delays the UART/RS232 forwarding.
static void btTxTask (void *)
{
  for (;;)
    {
      size_t len = 0;
      uint8_t *data = (uint8_t *) xRingbufferReceiveUpTo (btTxRingbuf, &len, portMAX_DELAY, BUFFER_SIZE);
      if (data == nullptr)
        continue;
      if (btRunning && SerialBT.hasClient ())
        SerialBT.write (data, len);
      vRingbufferReturnItem (btTxRingbuf, data);
    }
}

static void tryStartBluetooth (void)
{
  btLastStartAttemptMs = millis ();
  btRunning = SerialBT.begin (btDeviceName);
  if (! btRunning)
    SerialBT.end (); // release a half-initialized stack before the next attempt
  Serial.print ("Bluetooth: ");
  Serial.print (btRunning ? "started as " : "failed to start as ");
  Serial.println (btDeviceName);
}

void startBluetoothBridge (const char *deviceName)
{
  for (int num = 0; num < NUM_COM; ++num)
    for (int slot = 0; slot < MAX_NMEA_CLIENTS; ++slot)
      if (tcpClient[num][slot])
        tcpClient[num][slot].stop ();

  strncpy (btDeviceName, deviceName, sizeof(btDeviceName) - 1);
  btDeviceName[sizeof(btDeviceName) - 1] = 0;
  btRequested = true;

  btTxRingbuf = xRingbufferCreateStatic (BT_TX_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF,
                                         btTxRingbufStorage, &btTxRingbufStruct);
  xTaskCreateStatic (btTxTask, "bt_tx", BT_TX_TASK_STACK_SIZE, nullptr, 1,
                     btTxTaskStack, &btTxTaskTcb);

  tryStartBluetooth ();
}

bool bluetoothBridgeActive (void)
{
  return btRequested;
}

//!< Same data paths as the pre-WLAN-link sketch: Bluetooth -> UARTs, and
//!< every UART -> Bluetooth.
static void handleBluetoothBridge (void)
{
  if (! btRunning && (millis () - btLastStartAttemptMs >= BT_START_RETRY_MS))
    tryStartBluetooth ();

  if (btRunning && SerialBT.hasClient ())
    {
      uint16_t n = 0;
      while (SerialBT.available () && (n < BUFFER_SIZE - 1))
        netToUartBuf[n++] = SerialBT.read ();
      // not to COM[0], the debug console - see the RS232 echo comment below
      COM[1]->write (netToUartBuf, n);
      COM[2]->write (netToUartBuf, n);
    }

  for (int num = 0; num < NUM_COM; ++num)
    {
      if (! COM[num]->available ())
        continue;
      uartToNetLen[num] = 0;
      while (COM[num]->available () && (uartToNetLen[num] < BUFFER_SIZE - 1))
        uartToNetBuf[num][uartToNetLen[num]++] = COM[num]->read ();

      COM[1]->write (uartToNetBuf[num], uartToNetLen[num]); // echo to RS232 output
      // non-blocking: if btTxTask is behind (stalled peer), drop this chunk
      if (btRunning && SerialBT.hasClient ())
        xRingbufferSend (btTxRingbuf, uartToNetBuf[num], uartToNetLen[num], 0);
    }
}

void handleUartBridge (void)
{
  if (btRequested)
    {
      handleBluetoothBridge ();
      return;
    }
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
                  tcpClient[num][slot].setTimeout (NMEA_CLIENT_TIMEOUT_MS);
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
              {
                // A short write (the client isn't ACKing within
                // NMEA_CLIENT_TIMEOUT_MS - e.g. a phone's WiFi power-save
                // pausing delivery) means this client is stalled, not just
                // slow. Drop it now rather than let the same client stall
                // this same write, on this same port, every loop() pass -
                // that would otherwise keep blocking the web server and
                // every other NMEA client/port for NMEA_CLIENT_TIMEOUT_MS
                // at a time for as long as the stall lasts.
                size_t written = tcpClient[num][slot].write (uartToNetBuf[num], uartToNetLen[num]);
                if (written < uartToNetLen[num])
                  tcpClient[num][slot].stop ();
              }

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
