// Larus Sensor ESP32 firmware
// WiFi AP + web server for STM32 firmware update and *.lrsx log file
// management (upload/list/download/delete/format, relayed over SPI2 to the
// STM32 - see documentation/wlan_link.md in the sw_sensor
// repository), plus the 3x UART <-> WiFi TCP bridge, running concurrently
// with the WiFi AP.
//
// Bluetooth Classic support is temporarily removed (see uart_bridge.h) so
// the sketch fits an OTA-capable partition scheme; WiFi-only for now.
//
// UNTESTED ON HARDWARE. See the design doc above for what has and hasn't
// been validated.
//
// Disclaimer: Don't use for life support systems or any other situations
// where system failure may affect user or environmental safety.

#include "ap_identity.h"
#include "wifi_config.h"
#include "web_server.h"
#include "wlan_link_client.h"
#include "uart_bridge.h"

void setup ()
{
  // Without an explicit begin() here, the UART driver (ring buffers,
  // interrupt-driven TX) is never actually initialized by the Arduino
  // runtime - relying only on whatever the bootloader left configured,
  // which can silently drop the first Serial.print() calls after a reset.
  //
  // 115200, matching the bootloader's own default and UART_BAUD0
  // (uart_bridge.cpp): setupUartBridge() below reconfigures this same
  // Serial/UART0 object to that same baud as "UART bridge channel 0" -
  // its TX pin, GPIO1, is the same wire as this USB-serial debug link -
  // so keep both in sync, or everything printed to Serial from that
  // point on is silently sent at the other rate and reads as garbage on
  // a monitor left at this one.
  Serial.begin (115200);

  // Drive IO5 (WLAN_LINK_PIN_FRAME_START) to its idle-high state and bring
  // up the SPI bus as early as possible - before setupWifi() and
  // setupWebServer() below, which can take real time (WiFi/AP bring-up).
  // IO5 is one of the ESP32's own SPI-boot strapping pins and is
  // undefined/glitchy until firmware explicitly drives it; the STM32
  // boots much faster and watches this same line via EXTI from early in
  // its own boot. Minimizing the time between "this firmware starts
  // running" and "the pin is in a known, driven state" shrinks (doesn't
  // eliminate - the ESP32 boot ROM phase before this point is out of
  // firmware's control either way) the window in which a glitch here
  // could reach the STM32 while its own SPI2 link is still initializing.
  // See documentation/wlan_link.md and configure_wlan_link_gpio_and_spi()
  // (STM32 side, wlan_link_handler.cpp) for the matching STM32-side fix.
  wlanLink.begin ();

  char ssid[AP_IDENTITY_SSID_MAX_LEN];
  char password[AP_IDENTITY_PASSWORD_MAX_LEN];
  computeApIdentity (ssid, sizeof(ssid), password, sizeof(password));

  setupWifi (ssid, password);
  setupWebServer ();
  setupUartBridge ();

  Serial.println ("Larus Sensor ESP32 ready");
  Serial.print ("WiFi AP SSID: ");
  Serial.println (ssid);
  Serial.print ("WiFi AP password: ");
  Serial.println (password);
}

void loop ()
{
  handleWifiLoop ();
  handleWebServerClient ();
  handleUartBridge ();
}
