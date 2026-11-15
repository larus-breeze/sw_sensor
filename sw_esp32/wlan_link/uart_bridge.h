/** *****************************************************************************
 * @file    uart_bridge.h
 * @brief   3x UART <-> WiFi TCP / Bluetooth Classic bridge
 *
 * Carries over the original sketch's UART bridging (3 UART ports:
 * USB/programmer, RS232, STM32 telemetry): over WiFi TCP while WiFi is up,
 * over Bluetooth Classic (SPP) once WiFi has been switched off after the
 * idle timeout (see wifi_config.h). See documentation/wlan_link.md,
 * "WiFi to Bluetooth handover".
 *
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#ifndef UART_BRIDGE_H_
#define UART_BRIDGE_H_

void setupUartBridge (void);
void handleUartBridge (void); // call from loop()

//!< switches the bridge from WiFi TCP to Bluetooth Classic SPP, advertised
//!< as deviceName. Call once, after WiFi has been switched off.
void startBluetoothBridge (const char *deviceName);
bool bluetoothBridgeActive (void);

#endif /* UART_BRIDGE_H_ */
