#include "wifi_config.h"
#include <string.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <esp_netif.h>
#include <esp_netif_net_stack.h> // esp_netif_get_netif_impl() - not declared by esp_netif.h alone
extern "C" {
#include "lwip/etharp.h"
#include "lwip/priv/tcpip_priv.h" // LOCK_TCPIP_CORE()/UNLOCK_TCPIP_CORE()
}

//!< the web UI is also reachable as http://larus.local instead of an IP
#define MDNS_HOSTNAME "larus"

#define NVS_NAMESPACE   "wificfg"
#define NVS_KEY_SSID    "ssid"
#define NVS_KEY_PASSWORD "password"
#define NVS_KEY_AP_AUTO_OFF "apautooff"

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

//!< access point auto-off: idle time without any associated station
#define AP_AUTO_OFF_IDLE_MS          (5UL * 60UL * 1000UL)

static char storedApSsid[WIFI_CONFIG_SSID_MAX_LEN];
static char storedApPassword[WIFI_CONFIG_PASSWORD_MAX_LEN];
static char staSsid[WIFI_CONFIG_SSID_MAX_LEN];
static char staPassword[WIFI_CONFIG_PASSWORD_MAX_LEN];

static wifi_config_mode_t currentMode = WIFI_CONFIG_MODE_AP;

static unsigned long staLostSinceMs = 0;     //!< 0 while connected / not yet tracking a loss
static unsigned long lastRetryAttemptMs = 0;
static bool retryInProgress = false;
static unsigned long retryStartedMs = 0;

static bool apAutoOff = false;           //!< switch the AP off after AP_AUTO_OFF_IDLE_MS without a client
static bool apSwitchedOff = false;       //!< AP mode, but radio off after the idle timeout
static unsigned long apIdleSinceMs = 0;  //!< last time the AP had a client (or was (re)started)

static void loadStaConfig (void)
{
  Preferences preferences;
  preferences.begin (NVS_NAMESPACE, true); // read-only
  String ssid = preferences.getString (NVS_KEY_SSID, "");
  String password = preferences.getString (NVS_KEY_PASSWORD, "");
  apAutoOff = preferences.getBool (NVS_KEY_AP_AUTO_OFF, false);
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

// Experimental workaround for a long-standing, never officially resolved
// class of esp-idf/arduino-esp32 softAP issues (e.g.
// https://github.com/espressif/esp-idf/issues/6107,
// https://github.com/espressif/arduino-esp32/issues/4294): a station
// (observed with phones, not laptops) reconnecting to this AP can end up
// with a stale ARP entry for 192.168.4.1, making the AP look completely
// unreachable (ping and TCP both) until something happens to refresh it.
// The softAP netif already sends a gratuitous ARP on its own, but only
// about once a minute - broadcasting one immediately whenever a station
// associates means a client that just (re)connected doesn't have to wait
// up to that long for its ARP cache to catch up.
static void sendGratuitousArpForAp (void)
{
  esp_netif_t *apNetif = esp_netif_get_handle_from_ifkey ("WIFI_AP_DEF");
  if (apNetif == nullptr)
    return;
  struct netif *lwipNetif = (struct netif *) esp_netif_get_netif_impl (apNetif);
  if (lwipNetif == nullptr)
    return;

  // etharp_gratuitous() touches lwIP's internal state directly, bypassing
  // the sequential API - lwIP asserts if that happens from outside the
  // tcpip thread without holding this lock first (WiFi.onEvent()'s
  // callback runs on the Arduino event task, not the tcpip thread).
  // lwIP's own periodic gratuitous-ARP timer doesn't need this itself,
  // since it already runs on the tcpip thread.
  LOCK_TCPIP_CORE ();
  etharp_gratuitous (lwipNetif);
  UNLOCK_TCPIP_CORE ();
}

static void onWifiEvent (WiFiEvent_t event)
{
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED)
    sendGratuitousArpForAp ();
}

//!< Undoes startAccessPoint()'s reduced TX power once WiFi client mode
//!< actually takes over the radio - that network isn't necessarily as
//!< close as this AP's own clients are.
static void restoreFullTxPowerForStation (void)
{
  WiFi.setTxPower (WIFI_POWER_19_5dBm);
}

static void startAccessPoint (void)
{
  IPAddress ip (192, 168, 4, 1);
  IPAddress netmask (255, 255, 255, 0);

  WiFi.mode (WIFI_AP);
  WiFi.softAP (storedApSsid, storedApPassword);
  delay (200);
  WiFi.softAPConfig (ip, ip, netmask);

  // Sensor and client are typically <1m apart (in-cockpit use) - full TX
  // power isn't needed at that range. This is a whole-radio setting, not
  // AP-interface-specific - restoreFullTxPowerForStation() undoes it
  // whenever WiFi client mode actually takes over the radio, since that
  // network (e.g. a home WiFi) isn't necessarily anywhere near this close.
  WiFi.setTxPower (WIFI_POWER_13dBm);

  currentMode = WIFI_CONFIG_MODE_AP;
  apSwitchedOff = false;
  apIdleSinceMs = millis ();
  staLostSinceMs = 0;
  // next background retry is due one full interval from *now*, not
  // immediately - we may have just failed a connection attempt
  lastRetryAttemptMs = millis ();

  restartMdns ();

  Serial.println ("WiFi: access point mode");
}

static void switchApOff (void)
{
  MDNS.end ();
  WiFi.softAPdisconnect (true);
  WiFi.mode (WIFI_OFF);
  apSwitchedOff = true;
  lastRetryAttemptMs = millis (); // background STA retries (if configured) continue, STA-only
  Serial.println ("WiFi: access point switched off - no client for the configured time");
}

//!< Station count (not the web server's TCP clients) is what matters: any
//!< associated device - web UI, NMEA/UART TCP bridge, or just idle - keeps
//!< the AP alive.
static void handleApAutoOff (unsigned long now)
{
  if (apSwitchedOff || retryInProgress || ! apAutoOff)
    return;
  if (WiFi.softAPgetStationNum () > 0)
    {
      apIdleSinceMs = now;
      return;
    }
  if (now - apIdleSinceMs >= AP_AUTO_OFF_IDLE_MS)
    switchApOff ();
}

void setupWifi (const char *apSsid, const char *apPassword)
{
  strncpy (storedApSsid, apSsid, sizeof(storedApSsid) - 1);
  storedApSsid[sizeof(storedApSsid) - 1] = 0;
  strncpy (storedApPassword, apPassword, sizeof(storedApPassword) - 1);
  storedApPassword[sizeof(storedApPassword) - 1] = 0;

  loadStaConfig ();

  WiFi.onEvent (onWifiEvent);

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
      restoreFullTxPowerForStation ();
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

  // currentMode == WIFI_CONFIG_MODE_AP
  handleApAutoOff (now);

  // Periodically retry the configured network in the background, without
  // dropping the access point unless the retry actually succeeds.
  if (staSsid[0] == 0)
    return; // nothing configured to retry

  if (retryInProgress)
    {
      if (WiFi.status () == WL_CONNECTED)
        {
          retryInProgress = false;
          currentMode = WIFI_CONFIG_MODE_STA;
          staLostSinceMs = 0;
          if (! apSwitchedOff)
            WiFi.softAPdisconnect (true);
          apSwitchedOff = false;
          WiFi.mode (WIFI_STA);
          restoreFullTxPowerForStation ();
          restartMdns ();
          Serial.print ("WiFi: reconnected, IP ");
          Serial.println (WiFi.localIP ());
          return;
        }
      if (now - retryStartedMs >= STA_RETRY_CONNECT_TIMEOUT_MS)
        {
          retryInProgress = false;
          if (apSwitchedOff)
            {
              WiFi.disconnect ();
              WiFi.mode (WIFI_OFF);
              lastRetryAttemptMs = now;
            }
          else
            {
              // A failed STA probe isn't AP activity - keep the idle timer,
              // or a configured-but-absent network (retried every
              // STA_RETRY_INTERVAL_MS) keeps the AP from ever switching off.
              unsigned long idleSinceMs = apIdleSinceMs;
              startAccessPoint (); // undo the WIFI_AP_STA probe, resume pure AP
              apIdleSinceMs = idleSinceMs;
            }
        }
      return;
    }

  if (now - lastRetryAttemptMs >= STA_RETRY_INTERVAL_MS)
    {
      retryInProgress = true;
      retryStartedMs = now;
      // access point (unless auto-switched off) keeps serving existing
      // clients during the probe
      WiFi.mode (apSwitchedOff ? WIFI_STA : WIFI_AP_STA);
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

void saveApAutoOff (bool enabled)
{
  Preferences preferences;
  preferences.begin (NVS_NAMESPACE, false);
  preferences.putBool (NVS_KEY_AP_AUTO_OFF, enabled);
  preferences.end ();

  apAutoOff = enabled;
  apIdleSinceMs = millis (); // idle time counts from now
}

bool wifiConfigApAutoOff (void)
{
  return apAutoOff;
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
