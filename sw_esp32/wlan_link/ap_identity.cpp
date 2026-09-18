/** *****************************************************************************
 * @file    ap_identity.cpp
 * @brief   WLAN AP SSID + password derivation
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#include "ap_identity.h"
#include <string.h>
#include <esp_system.h>
#include <WiFi.h>
#include <Preferences.h>

#define NVS_NAMESPACE     "apident"
#define NVS_KEY_PASSWORD  "password"

static const char CROCKFORD_BASE32_ALPHABET[] = "0123456789ABCDEFGHJKMNPQRSTVWXYZ"; // no 0/O, 1/I/L ambiguity

//!< encodes dataLen bytes into Crockford base32 chars into out (NUL-terminated,
//!< truncated if it wouldn't fit in outCapacity)
static void base32EncodeCrockford (const uint8_t *data, size_t dataLen, char *out, size_t outCapacity)
{
  uint32_t bitBuffer = 0;
  int bitsInBuffer = 0;
  size_t outIndex = 0;

  for (size_t i = 0; i < dataLen; ++i)
    {
      bitBuffer = (bitBuffer << 8) | data[i];
      bitsInBuffer += 8;
      while (bitsInBuffer >= 5)
        {
          bitsInBuffer -= 5;
          uint8_t idx = (uint8_t) ((bitBuffer >> bitsInBuffer) & 0x1F);
          if ((outIndex + 1) < outCapacity)
            out[outIndex++] = CROCKFORD_BASE32_ALPHABET[idx];
        }
    }
  if (bitsInBuffer > 0)
    {
      uint8_t idx = (uint8_t) ((bitBuffer << (5 - bitsInBuffer)) & 0x1F);
      if ((outIndex + 1) < outCapacity)
        out[outIndex++] = CROCKFORD_BASE32_ALPHABET[idx];
    }
  if (outIndex < outCapacity)
    out[outIndex] = 0;
}

//!< fills buf[0..len) with bytes from the ESP32's hardware RNG. Per
//!< Espressif's documentation this is cryptographically strong as long as
//!< WiFi and/or Bluetooth are enabled - the caller (below) makes sure the
//!< WiFi radio is already on before this runs.
static void fillRandomBytes (uint8_t *buf, size_t len)
{
  size_t i = 0;
  while (i < len)
    {
      uint32_t r = esp_random ();
      size_t n = (len - i) < sizeof(r) ? (len - i) : sizeof(r);
      memcpy (buf + i, &r, n);
      i += n;
    }
}

//!< loads the AP password from NVS, or - on first boot, when none is
//!< stored yet - generates a fresh random one and persists it. Must
//!< persist: a password regenerated on every boot would invalidate any
//!< printed/scanned QR label the moment the device restarts.
static void loadOrGeneratePassword (char *passwordOut, size_t passwordCapacity)
{
  Preferences preferences;
  preferences.begin (NVS_NAMESPACE, false); // read-write

  String stored = preferences.getString (NVS_KEY_PASSWORD, "");
  if (stored.length () > 0)
    {
      strncpy (passwordOut, stored.c_str (), passwordCapacity - 1);
      passwordOut[passwordCapacity - 1] = 0;
      preferences.end ();
      return;
    }

  // First boot (or NVS was erased). The RNG's entropy is documented as
  // strong only once the WiFi (or BT) radio is enabled - make sure of
  // that here rather than relying on the caller's ordering; startAccessPoint()
  // (wifi_config.cpp) calls WiFi.mode(WIFI_AP) again right after this
  // returns, which is harmless/idempotent.
  WiFi.mode (WIFI_AP);

  uint8_t randomBytes[10]; // 80 bits = exactly 16 Crockford base32 chars, no padding
  fillRandomBytes (randomBytes, sizeof(randomBytes));
  base32EncodeCrockford (randomBytes, sizeof(randomBytes), passwordOut, passwordCapacity);

  preferences.putString (NVS_KEY_PASSWORD, passwordOut);
  preferences.end ();

  Serial.println ("AP identity: generated new WLAN AP password (first boot)");
}

void computeApIdentity (char *ssidOut, size_t ssidCapacity, char *passwordOut, size_t passwordCapacity)
{
  uint64_t mac = ESP.getEfuseMac();
  uint16_t uid = (uint16_t) (mac & 0xFFFFu);
  snprintf (ssidOut, ssidCapacity, "Larus_%X", uid); // matches the existing Bluetooth device name scheme

  loadOrGeneratePassword (passwordOut, passwordCapacity);
}

String buildWifiQrPayload (const char *ssid, const char *password)
{
  String payload = "WIFI:S:";
  payload += ssid;
  payload += ";T:WPA;P:";
  payload += password;
  payload += ";;";
  return payload;
}
