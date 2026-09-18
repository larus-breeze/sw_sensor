/** *****************************************************************************
 * @file    wifi_config.h
 * @brief   Optional WiFi client (station) mode, persisted in NVS, with
 *          automatic fallback to the sensor's own access point
 *
 * The sensor normally runs as its own WiFi access point (see
 * ap_identity.h). This lets it instead join an existing WiFi network as a
 * client, e.g. so the web UI and log files are reachable over a home/hangar
 * network without joining the sensor's own AP. If the configured network
 * can't be reached (at boot, or later if the connection is lost), the
 * sensor automatically falls back to its own access point, and keeps
 * retrying the configured network in the background - AP clients aren't
 * dropped during a background retry (AP+STA run concurrently for the few
 * seconds each attempt takes), only committed away from once a retry
 * actually succeeds.
 *
 * @license This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#ifndef WIFI_CONFIG_H_
#define WIFI_CONFIG_H_

#include <Arduino.h>

#define WIFI_CONFIG_SSID_MAX_LEN       33 //!< 32 chars (WiFi SSID limit) + NUL
#define WIFI_CONFIG_PASSWORD_MAX_LEN   64 //!< 63 chars (WPA2 passphrase limit) + NUL

enum wifi_config_mode_t
{
  WIFI_CONFIG_MODE_AP,  //!< running the sensor's own access point (default / fallback)
  WIFI_CONFIG_MODE_STA, //!< successfully joined the configured network as a client
};

//!< brings up WiFi: starts the access point (apSsid/apPassword, as before),
//!< then - if a client network is configured - also tries to join it,
//!< blocking up to a bounded timeout; on success, switches away from the
//!< access point to station mode. Call once from setup(), after
//!< computeApIdentity().
void setupWifi (const char *apSsid, const char *apPassword);

//!< call from loop(): monitors the station connection (falls back to the
//!< access point if it's lost) and, while in access point mode with a
//!< client network configured, periodically retries joining it in the
//!< background.
void handleWifiLoop (void);

//!< persists new station credentials (pass ssid="" to disable/forget the
//!< configured network) and reboots to apply them - the caller should
//!< already have responded to the HTTP request before calling this, since
//!< it never returns.
void saveWifiConfigAndRestart (const char *ssid, const char *password);

wifi_config_mode_t wifiConfigCurrentMode (void);
bool wifiConfigStaConfigured (void);
String wifiConfigStaSsid (void); //!< configured SSID, or "" - never the password

#endif /* WIFI_CONFIG_H_ */
