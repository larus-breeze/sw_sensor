/** *****************************************************************************
 * @file    ap_identity.h
 * @brief   WLAN AP SSID + password derivation
 *
 * See documentation/wlan_link.md, "AP identity: SSID and password".
 *
 * SSID: unchanged from the existing sketch's scheme - "Larus_<uid>" from
 * the low 16 bits of the chip's factory-programmed MAC (ESP.getEfuseMac()),
 * matching the Bluetooth device name. Not secret, doesn't need to be.
 *
 * Password: a random 80-bit value (ESP32 hardware RNG, esp_random()),
 * Crockford base32-encoded (16 chars, no ambiguous 0/O 1/I/L), generated
 * once on first boot and persisted in NVS from then on. Deliberately *not*
 * derived from the chip's MAC: the AP's own BSSID is broadcast in the
 * clear in every beacon frame and is a fixed, publicly documented offset
 * from that same MAC (Espressif's SoftAP-MAC-from-base-MAC scheme) - so
 * anyone in radio range can recover the full MAC without even associating,
 * no scan of the SSID required. A previous version of this file derived
 * the password as HMAC-SHA256(pepper, MAC); since the pepper also ships in
 * this public repository, that construction was fully invertible from the
 * BSSID alone and gave no real protection. A random, per-device,
 * NVS-persisted secret has no such relationship to anything broadcast over
 * the air, so this weakness doesn't apply to it.
 *
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#ifndef AP_IDENTITY_H_
#define AP_IDENTITY_H_

#include <Arduino.h>

#define AP_IDENTITY_SSID_MAX_LEN       16 //!< "Larus_" + up to 4 hex digits + NUL
#define AP_IDENTITY_PASSWORD_MAX_LEN   17 //!< 16 base32 chars + NUL

//!< fills ssidOut/passwordOut (call with buffers at least
//!< AP_IDENTITY_SSID_MAX_LEN / AP_IDENTITY_PASSWORD_MAX_LEN bytes)
void computeApIdentity (char *ssidOut, size_t ssidCapacity, char *passwordOut, size_t passwordCapacity);

//!< the standard WiFi-network QR payload ("WIFI:S:<ssid>;T:WPA;P:<password>;;")
//!< for a phone's camera app to join by scanning - actual QR *image*
//!< rendering isn't implemented here, but sw_esp32/scripts/generate_ap_label.py
//!< renders a printable label (QR code + text) from the SSID/password this
//!< firmware prints over serial at boot, building this exact payload string
//!< independently on the host side; this string can also be fed into any
//!< other external QR generator, or just read/typed directly.
String buildWifiQrPayload (const char *ssid, const char *password);

#endif /* AP_IDENTITY_H_ */
