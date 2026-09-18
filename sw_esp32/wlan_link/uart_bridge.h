/** *****************************************************************************
 * @file    uart_bridge.h
 * @brief   3x UART <-> WiFi TCP bridge
 *
 * Carries over the existing sketch's UART/TCP bridging (3 UART ports:
 * USB/programmer, RS232, STM32 telemetry). Bluetooth Classic support
 * (BluetoothSerial) is temporarily removed - it pushed the compiled sketch
 * past the app-partition budget of every OTA-capable partition scheme,
 * and OTA updatability was judged more important for now. See
 * documentation/wlan_link.md, "ESP32 connectivity: WiFi + Bluetooth
 * coexistence".
 *
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#ifndef UART_BRIDGE_H_
#define UART_BRIDGE_H_

void setupUartBridge (void);
void handleUartBridge (void); // call from loop()

#endif /* UART_BRIDGE_H_ */
