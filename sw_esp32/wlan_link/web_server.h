/** *****************************************************************************
 * @file    web_server.h
 * @brief   HTTP server: STM32 + ESP32 firmware update, log file browser,
 *          SD card format
 *
 * See documentation/wlan_link.md, "ESP32 web UI".
 *
 * STM32 firmware uploads are relayed to the STM32 chunk-by-chunk as they
 * arrive from the browser, not buffered whole first - the STM32-side
 * upload already has its own end-to-end CRC32 check
 * (wlan_link_handler.cpp's handle_upload_end()).
 *
 * The ESP32's own firmware is updated separately (/update_esp32) via the
 * Arduino core's Update library, into the free OTA_x partition.
 *
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#ifndef WEB_SERVER_H_
#define WEB_SERVER_H_

#include <Arduino.h>

void setupWebServer (void); // call after setupWifi() has brought up networking
void handleWebServerClient (void); // call from loop()

#endif /* WEB_SERVER_H_ */
