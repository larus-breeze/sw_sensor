#include "wifi_config.h"
#include <string.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ESPmDNS.h>

//!< the web UI is also reachable as http://larus.local instead of an IP
#define MDNS_HOSTNAME "larus"

#define NVS_NAMESPACE   "wificfg"
#define NVS_KEY_SSID    "ssid"
#define NVS_KEY_PASSWORD "password"

//!< how long setupWifi() blocks waiting for the initial connection attempt
//!< to the configured network - the access point is already up throughout
//!< this wait (see setupWifi()), so this doesn't delay reachability.
#define STA_CONNECT_TIMEOUT_MS       15000u

//!< how long a background retry (handleWifiLoop(), while in AP mode) waits
//!< before giving up and resuming pure access point mode
#define STA_RETRY_CONNECT_TIMEOUT_MS  8000u

//!< how often to retry the configured network in the background while
//!< stuck in access point mode
#define STA_RETRY_INTERVAL_MS        (60UL * 1000UL)

//!< how long an established station connection may report "not connected"
//!< before falling back to the access point - rides out brief hiccups the
//!< WiFi stack's own auto-reconnect would otherwise recover from on its own
#define STA_LOST_GRACE_MS            10000u

static char storedApSsid[WIFI_CONFIG_SSID_MAX_LEN];
static char storedApPassword[WIFI_CONFIG_PASSWORD_MAX_LEN];
static char staSsid[WIFI_CONFIG_SSID_MAX_LEN];
static char staPassword[WIFI_CONFIG_PASSWORD_MAX_LEN];

static wifi_config_mode_t currentMode = WIFI_CONFIG_MODE_AP;

static unsigned long staLostSinceMs = 0;     //!< 0 while connected / not yet tracking a loss
static unsigned long lastRetryAttemptMs = 0;
static bool retryInProgress = false;
static unsigned long retryStartedMs = 0;

static void loadStaConfig (void)
{
  Preferences preferences;
  preferences.begin (NVS_NAMESPACE, true); // read-only
  String ssid = preferences.getString (NVS_KEY_SSID, "");
  String password = preferences.getString (NVS_KEY_PASSWORD, "");
  preferences.end ();

  strncpy (staSsid, ssid.c_str (), sizeof(staSsid) - 1);
  staSsid[sizeof(staSsid) - 1] = 0;
  strncpy (staPassword, password.c_str (), sizeof(staPassword) - 1);
  staPassword[sizeof(staPassword) - 1] = 0;
}

//!< (Re)starts the mDNS responder. Needed after every AP<->STA transition,
//!< not just once at boot - the underlying network interface changes, and
//!< a responder bound to the old one stops working. MDNS.end() first makes
//!< repeated calls safe instead of stacking duplicate registrations.
static void restartMdns (void)
{
  MDNS.end ();
  if (MDNS.begin (MDNS_HOSTNAME))
    MDNS.addService ("http", "tcp", 80);
}

static void startAccessPoint (void)
{
  IPAddress ip (192, 168, 4, 1);
  IPAddress netmask (255, 255, 255, 0);

  WiFi.mode (WIFI_AP);
  WiFi.softAP (storedApSsid, storedApPassword);
  delay (200);
  WiFi.softAPConfig (ip, ip, netmask);

  currentMode = WIFI_CONFIG_MODE_AP;
  staLostSinceMs = 0;
  // next background retry is due one full interval from *now*, not
  // immediately - we may have just failed a connection attempt
  lastRetryAttemptMs = millis ();

  restartMdns ();

  Serial.println ("WiFi: access point mode");
}

void setupWifi (const char *apSsid, const char *apPassword)
{
  strncpy (storedApSsid, apSsid, sizeof(storedApSsid) - 1);
  storedApSsid[sizeof(storedApSsid) - 1] = 0;
  strncpy (storedApPassword, apPassword, sizeof(storedApPassword) - 1);
  storedApPassword[sizeof(storedApPassword) - 1] = 0;

  loadStaConfig ();

  // Access point first, always - reachable immediately, and stays that way
  // even if the join attempt below takes the full timeout to fail.
  startAccessPoint ();

  if (staSsid[0] == 0)
    return; // no client network configured

  Serial.print ("WiFi: trying to join '");
  Serial.print (staSsid);
  Serial.println ("'...");

  WiFi.mode (WIFI_AP_STA); // keep serving the access point while probing
  WiFi.begin (staSsid, staPassword);

  unsigned long start = millis ();
  while ((WiFi.status () != WL_CONNECTED) && (millis () - start < STA_CONNECT_TIMEOUT_MS))
    delay (250);

  if (WiFi.status () == WL_CONNECTED)
    {
      currentMode = WIFI_CONFIG_MODE_STA;
      WiFi.softAPdisconnect (true);
      WiFi.mode (WIFI_STA);
      restartMdns ();
      Serial.print ("WiFi: connected, IP ");
      Serial.println (WiFi.localIP ());
    }
  else
    {
      Serial.println ("WiFi: could not join configured network, staying in access point mode");
      startAccessPoint (); // drop the STA attempt, re-affirm pure AP mode
    }
}

void handleWifiLoop (void)
{
  unsigned long now = millis ();

  if (currentMode == WIFI_CONFIG_MODE_STA)
    {
      if (WiFi.status () == WL_CONNECTED)
        {
          staLostSinceMs = 0;
          return;
        }
      if (staLostSinceMs == 0)
        {
          staLostSinceMs = now; // start of grace period
          return;
        }
      if (now - staLostSinceMs >= STA_LOST_GRACE_MS)
        {
          Serial.println ("WiFi: lost configured network, falling back to access point");
          startAccessPoint ();
        }
      return;
    }

  // currentMode == WIFI_CONFIG_MODE_AP: periodically retry the configured
  // network in the background, without dropping the access point unless
  // the retry actually succeeds.
  if (staSsid[0] == 0)
    return; // nothing configured to retry

  if (retryInProgress)
    {
      if (WiFi.status () == WL_CONNECTED)
        {
          retryInProgress = false;
          currentMode = WIFI_CONFIG_MODE_STA;
          staLostSinceMs = 0;
          WiFi.softAPdisconnect (true);
          WiFi.mode (WIFI_STA);
          restartMdns ();
          Serial.print ("WiFi: reconnected, IP ");
          Serial.println (WiFi.localIP ());
          return;
        }
      if (now - retryStartedMs >= STA_RETRY_CONNECT_TIMEOUT_MS)
        {
          retryInProgress = false;
          startAccessPoint (); // undo the WIFI_AP_STA probe, resume pure AP
        }
      return;
    }

  if (now - lastRetryAttemptMs >= STA_RETRY_INTERVAL_MS)
    {
      retryInProgress = true;
      retryStartedMs = now;
      WiFi.mode (WIFI_AP_STA); // access point keeps serving existing clients during the probe
      WiFi.begin (staSsid, staPassword);
    }
}

void saveWifiConfigAndRestart (const char *ssid, const char *password)
{
  Preferences preferences;
  preferences.begin (NVS_NAMESPACE, false);
  preferences.putString (NVS_KEY_SSID, ssid);
  preferences.putString (NVS_KEY_PASSWORD, password);
  preferences.end ();
  ESP.restart ();
}

wifi_config_mode_t wifiConfigCurrentMode (void)
{
  return currentMode;
}

bool wifiConfigStaConfigured (void)
{
  return staSsid[0] != 0;
}

String wifiConfigStaSsid (void)
{
  return String (staSsid);
}
