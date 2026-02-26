#include <Arduino.h>
#include "credentials.h"
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <SPIFFS.h>
#if defined(ESP32)
#include <esp_log.h>
#endif
#include <PubSubClient.h>
#include <time.h>
#include <HTTPClient.h>
// FreeRTOS for early printing task
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// (Adafruit IO removed) MQTT is used for telemetry/commands

// Ensure LED_BUILTIN is defined for this board
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// no Adafruit IO message handler

// Relay pin mapping (board: ESP32-S3-Relay-6CH)
// NOTE: GPIO1 is UART0 TX (serial) and using it may interfere with Serial output.
const uint8_t RELAY_PINS[6] = {1, 2, 41, 42, 45, 46};
// relayFeeds removed; we'll publish states via MQTT

// Early boot printing task
TaskHandle_t earlyPrintTaskHandle = NULL;
volatile bool stopEarlyPrint = false;

// forward declaration for task function
void earlyPrintTask(void *pvParameters);


// WS2812 (NeoPixel) RGB LED on GPIO38
#define RGB_PIN 38
#define NUM_RGB 1
Adafruit_NeoPixel strip(NUM_RGB, RGB_PIN, NEO_GRB + NEO_KHZ800);
// rgbFeed removed
// Preferences (NVS) instance used across the file
Preferences prefs;

// MQTT client (optional)
WiFiClient espClient;
PubSubClient mqtt(espClient);
const char *MQTT_BROKER = "test.mosquitto.org";
const uint16_t MQTT_PORT = 1883;

// Publish topics
const char *MQTT_TELEMETRY_TOPIC = "esp32s3/telemetry"; // JSON payload
const char *MQTT_COMMAND_TOPIC = "esp32s3/command/#";  // relay commands: esp32s3/command/relay1 -> payload 0/1

// Helper: handle incoming MQTT messages
// forward declarations for schedule functions used by mqttCallback
void loadScheduleFromFS();
void saveScheduleToFS();
void clearScheduleInMemory();
void initPendingOffs();

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("MQTT in ["); Serial.print(t); Serial.print("] -> "); Serial.println(msg);
  // Handle schedule payload published as CSV lines: ts,relay,duration\n...
  if (t == String("esp32s3/schedule")) {
    Serial.println("Received schedule payload; saving to /schedule.csv");
    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS mount failed; cannot save schedule");
      return;
    }
    File f = SPIFFS.open("/schedule.csv", FILE_WRITE);
    if (!f) {
      Serial.println("Failed to open /schedule.csv for writing");
      return;
    }
    f.print(msg);
    f.close();
    Serial.println("Schedule saved to /schedule.csv");
    // load into in-memory schedule
    loadScheduleFromFS();
    return;
  }
  // handle relay commands like topic ends with relayN
  if (t.startsWith("esp32s3/command/relay")) {
    int idx = t.substring(String("esp32s3/command/relay").length()).toInt();
    if (idx >= 1 && idx <= 6) {
      int val = msg.toInt();
      int pin = RELAY_PINS[idx-1];
      digitalWrite(pin, val ? HIGH : LOW);
      Serial.print("MQTT: set relay "); Serial.print(idx); Serial.print(" -> "); Serial.println(val);
      // publish back state on same topic without the wildcard
      char stateTopic[64];
      snprintf(stateTopic, sizeof(stateTopic), "esp32s3/state/relay%d", idx);
      mqtt.publish(stateTopic, digitalRead(pin) ? "1" : "0");
    }
  }
}

// Read device schedule URL from Preferences (key: "device_schedule_url" in namespace "cloud")
String getDeviceScheduleUrlFromPrefs() {
  prefs.begin("cloud", true);
  String url = prefs.getString("device_schedule_url", "");
  prefs.end();
  return url;
}

// POST schedule CSV contents to configured HTTP endpoint (device_schedule)
bool postScheduleToHttp(const String &contents) {
  String url = getDeviceScheduleUrlFromPrefs();
  if (url.length() == 0) return false;
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "text/plain");
  int code = http.POST((uint8_t*)contents.c_str(), contents.length());
  bool ok = (code >= 200 && code < 300);
  if (!ok) {
    Serial.print("HTTP POST schedule failed, code="); Serial.println(code);
  } else {
    Serial.println("HTTP POST schedule succeeded");
  }
  http.end();
  return ok;
}

// GET schedule CSV from configured HTTP endpoint and write to /schedule.csv
bool getScheduleFromHttpAndSave() {
  String url = getDeviceScheduleUrlFromPrefs();
  if (url.length() == 0) return false;
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  if (code != 200) {
    Serial.print("HTTP GET schedule failed, code="); Serial.println(code);
    http.end();
    return false;
  }
  String payload = http.getString();
  http.end();
  if (payload.length() == 0) return false;
  if (!SPIFFS.begin(true)) { Serial.println("SPIFFS mount failed; cannot save schedule from HTTP"); return false; }
  File f = SPIFFS.open("/schedule.csv", FILE_WRITE);
  if (!f) { Serial.println("Failed to open /schedule.csv for writing (HTTP)"); return false; }
  f.print(payload);
  f.close();
  Serial.println("Saved schedule.csv from HTTP endpoint");
  // load into memory
  loadScheduleFromFS();
  return true;
}

bool ensureMqttConnected() {
  if (mqtt.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  Serial.print("Connecting to MQTT broker "); Serial.print(MQTT_BROKER); Serial.print(":"); Serial.println(MQTT_PORT);
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  if (mqtt.connect("esp32s3-client")) {
    Serial.println("MQTT connected");
    // subscribe to command topics
    mqtt.subscribe("esp32s3/command/#");
    // subscribe to schedule topic so device receives schedule CSV payloads
    mqtt.subscribe("esp32s3/schedule");
    // publish current relay states so server and web UI are up-to-date
    for (uint8_t r = 1; r <= 6; ++r) {
      int pin = RELAY_PINS[r-1];
      const char *state = digitalRead(pin) ? "1" : "0";
      char stateTopic[64];
      snprintf(stateTopic, sizeof(stateTopic), "esp32s3/state/relay%u", r);
      mqtt.publish(stateTopic, state);
      Serial.print("MQTT: reported relay "); Serial.print(r); Serial.print(" -> "); Serial.println(state);
    }
    return true;
  }
  Serial.print("MQTT connect failed, state="); Serial.println(mqtt.state());
  return false;
}

// Adafruit IO removed: feeds are no longer created here. Use MQTT topics instead.
uint8_t current_r = 0, current_g = 0, current_b = 0;

// DHT22 sensors (AM2302) on pins 15 and 16
#define DHTPIN1 15
#define DHTPIN2 16
#define DHTTYPE DHT22
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);

// DHT read interval (ms) — changed to 2 minutes
const unsigned long DHT_INTERVAL = 120000; // 120000 ms = 2 minutes
unsigned long lastDHT = 0;
// Heartbeat interval (ms)
const unsigned long HEARTBEAT_INTERVAL = 5000;
unsigned long lastHeartbeat = 0;
// Controla si el firmware cicla automáticamente los relays (false = deshabilitado)
bool autoCycle = false;

// Start early print task (creates task that watches `stopEarlyPrint`)
void startEarlyBootPrints() {
  if (earlyPrintTaskHandle == NULL) {
    stopEarlyPrint = false;
    xTaskCreatePinnedToCore(earlyPrintTask, "earlyPrint", 2048, NULL, 1, &earlyPrintTaskHandle, 1);
  }
}

// Stop early print task by setting the flag; task exits and deletes itself
void stopEarlyBootPrints() {
  if (earlyPrintTaskHandle != NULL) {
    stopEarlyPrint = true;
  }
}

// Helper: publish current RGB as #RRGGBB
void publishRGB() {
  char buf[8];
  sprintf(buf, "#%02X%02X%02X", current_r, current_g, current_b);
  if (ensureMqttConnected()) {
    mqtt.publish("esp32s3/rgb", buf);
  }
}

// Parse color string: #RRGGBB or R,G,B
bool parseColorString(const String &s, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (s.length() == 7 && s.charAt(0) == '#') {
    long val = strtol(s.c_str() + 1, NULL, 16);
    r = (val >> 16) & 0xFF;
    g = (val >> 8) & 0xFF;
    b = val & 0xFF;
    return true;
  }
  int rr, gg, bb;
  if (sscanf(s.c_str(), "%d,%d,%d", &rr, &gg, &bb) == 3) {
    r = constrain(rr, 0, 255);
    g = constrain(gg, 0, 255);
    b = constrain(bb, 0, 255);
    return true;
  }
  return false;
}

// Adafruit IO feed handlers removed; remote control is via MQTT topics

// Serial command buffer and handler
String serialLine = "";
// Last WiFi scan result count (used by wscan/wjoin serial commands)
int lastWiFiScanCount = 0;

// Configuration portal state
WebServer configServer(80);
WebServer dashboardServer(80);
bool configPortalActive = false;
int configScanCount = 0;
bool dashboardActive = false;

// Simple circular buffer for recent samples
const int SAMPLE_BUFFER_SIZE = 200;
unsigned long sampleTimes[SAMPLE_BUFFER_SIZE];
float sample_t1[SAMPLE_BUFFER_SIZE];
float sample_h1[SAMPLE_BUFFER_SIZE];
float sample_t2[SAMPLE_BUFFER_SIZE];
float sample_h2[SAMPLE_BUFFER_SIZE];
int sampleIndex = 0;
int sampleCount = 0;
// Persist one sample every PERSIST_INTERVAL_MS to SPIFFS so server can retrieve it
const unsigned long PERSIST_INTERVAL_MS = 30UL * 60UL * 1000UL; // 30 minutes
unsigned long lastPersist = 0;
bool spiffsMountedForPersist = false;

// Schedule storage
const int SCHEDULE_MAX_EVENTS = 128;
struct ScheduleEvent { unsigned long ts; unsigned long long epoch_ms; uint8_t relay; unsigned long duration; bool used; bool repeat8; };
ScheduleEvent scheduleEvents[SCHEDULE_MAX_EVENTS];
int scheduleCount = 0;
// whether we have NTP/epoch time available
bool haveEpoch = false;

// helper to get current epoch ms if available
unsigned long long getCurrentEpochMs() {
  time_t now = time(NULL);
  if (now < 1600000000) return 0;
  unsigned long long ms = (unsigned long long)now * 1000ULL + (unsigned long)(millis() % 1000);
  return ms;
}

void clearScheduleInMemory() {
  for (int i = 0; i < SCHEDULE_MAX_EVENTS; ++i) { scheduleEvents[i].used = false; }
  scheduleCount = 0;
}

// Compact schedule array to remove unused entries and keep them contiguous
void compactSchedule() {
  int dst = 0;
  for (int src = 0; src < scheduleCount; ++src) {
    if (!scheduleEvents[src].used) continue;
    if (dst != src) scheduleEvents[dst] = scheduleEvents[src];
    dst++;
  }
  // mark remaining slots unused
  for (int i = dst; i < SCHEDULE_MAX_EVENTS; ++i) scheduleEvents[i].used = false;
  scheduleCount = dst;
}

// Helper to add a schedule event safely (returns true if added)
bool addScheduleEvent(unsigned long ts, unsigned long long epoch_ms, uint8_t relay, unsigned long duration, bool repeat8) {
  if (relay < 1 || relay > 6) return false;
  if (duration == 0) return false;
  if (scheduleCount >= SCHEDULE_MAX_EVENTS) {
    // try to compact and free space
    compactSchedule();
    if (scheduleCount >= SCHEDULE_MAX_EVENTS) return false;
  }
  scheduleEvents[scheduleCount].ts = ts;
  scheduleEvents[scheduleCount].epoch_ms = epoch_ms;
  scheduleEvents[scheduleCount].relay = relay;
  scheduleEvents[scheduleCount].duration = duration;
  scheduleEvents[scheduleCount].used = true;
  scheduleEvents[scheduleCount].repeat8 = repeat8;
  scheduleCount++;
  return true;
}

// Load schedule from /schedule.csv (lines: ts,relay,duration)
void loadScheduleFromFS() {
  if (!SPIFFS.begin(true)) { Serial.println("loadScheduleFromFS: SPIFFS mount failed"); return; }
  if (!SPIFFS.exists("/schedule.csv")) { Serial.println("loadScheduleFromFS: no schedule file"); clearScheduleInMemory(); return; }
  File f = SPIFFS.open("/schedule.csv", FILE_READ);
  if (!f) { Serial.println("loadScheduleFromFS: failed to open file"); return; }
  clearScheduleInMemory();
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (line.charAt(0) == '#') continue; // allow comments
    // split into fields separated by commas; allow 3 or 4 fields
    int fields[5];
    fields[0] = 0;
    int fieldIndex = 0;
    int last = 0;
    for (int i = 0; i <= line.length(); ++i) {
      if (i == line.length() || line.charAt(i) == ',') {
        if (fieldIndex < 4) {
          fields[fieldIndex++] = last;
        }
        last = i + 1;
      }
    }
    // now extract substrings safely
    // fallback: use strtok-like parsing for simplicity
    int numCommas = 0;
    for (int i = 0; i < line.length(); ++i) if (line.charAt(i) == ',') numCommas++;
    // expected 2 or 3 commas (3 or 4 fields)
    int expectedFields = (numCommas >= 3) ? 4 : (numCommas >= 2 ? 3 : 0);
    if (expectedFields < 3) continue;
    // parse by using indexOf to get positions
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = -1;
    if (expectedFields == 4) c3 = line.indexOf(',', c2 + 1);
    unsigned long long ets = 0ULL;
    uint8_t relay = 0;
    unsigned long dur = 0UL;
    bool repeat = false;
    // parse first field (ets)
    ets = (unsigned long long) strtoull(line.substring(0, c1).c_str(), NULL, 10);
    // parse relay
    relay = (uint8_t) atoi(line.substring(c1 + 1, c2).c_str());
    // parse duration and optional repeat
    if (expectedFields == 3) {
      dur = (unsigned long) strtoul(line.substring(c2 + 1).c_str(), NULL, 10);
    } else {
      dur = (unsigned long) strtoul(line.substring(c2 + 1, c3).c_str(), NULL, 10);
      repeat = atoi(line.substring(c3 + 1).c_str()) ? true : false;
    }
    // validate parsed values
    if (relay < 1 || relay > 6) continue;
    if (dur == 0) continue;
    unsigned long targetMs = 0UL;
    unsigned long long epochStore = 0ULL;
    if (ets > 1600000000000ULL) {
      epochStore = ets;
      targetMs = 0UL;
    } else {
      targetMs = (unsigned long) ets;
    }
    addScheduleEvent(targetMs, epochStore, relay, dur, repeat);
  }
  f.close();
  Serial.print("Loaded schedule events: "); Serial.println(scheduleCount);
  // Publish the saved schedule file contents so external servers/UIs can sync
  if (ensureMqttConnected()) {
    if (SPIFFS.exists("/schedule.csv")) {
      File rf = SPIFFS.open("/schedule.csv", FILE_READ);
      if (rf) {
        String contents = "";
        while (rf.available()) {
          contents += rf.readStringUntil('\n');
          contents += '\n';
        }
        rf.close();
        if (contents.length() > 0) {
          mqtt.publish("esp32s3/schedule/loaded", contents.c_str());
          Serial.println("Published schedule/loaded via MQTT");
          // Also attempt to post to HTTP device_schedule endpoint if configured
          if (getDeviceScheduleUrlFromPrefs().length() > 0) {
            if (postScheduleToHttp(contents)) {
              Serial.println("Posted schedule/loaded to HTTP endpoint (on load)");
            }
          }
        }
      }
    }
  }
}

// Save remaining schedule (non-executed) back to FS
void saveScheduleToFS() {
  if (!SPIFFS.begin(true)) { Serial.println("saveScheduleToFS: SPIFFS mount failed"); return; }
  // compact before saving so file doesn't contain holes
  compactSchedule();
  File f = SPIFFS.open("/schedule.csv", FILE_WRITE);
  if (!f) { Serial.println("saveScheduleToFS: failed to open file"); return; }
  for (int i = 0; i < scheduleCount; ++i) {
    if (!scheduleEvents[i].used) continue;
    char line[128];
    if (scheduleEvents[i].epoch_ms > 0ULL) {
      // write epoch-ms form
      snprintf(line, sizeof(line), "%llu,%u,%lu,%d\n", scheduleEvents[i].epoch_ms, (unsigned)scheduleEvents[i].relay, scheduleEvents[i].duration, scheduleEvents[i].repeat8 ? 1 : 0);
    } else {
      snprintf(line, sizeof(line), "%lu,%u,%lu,%d\n", scheduleEvents[i].ts, (unsigned)scheduleEvents[i].relay, scheduleEvents[i].duration, scheduleEvents[i].repeat8 ? 1 : 0);
    }
    f.print(line);
  }
  f.close();
  // After saving, publish the file contents so external servers/UIs can sync
  if (ensureMqttConnected()) {
    File rf = SPIFFS.open("/schedule.csv", FILE_READ);
    if (rf) {
      String contents = "";
      while (rf.available()) {
        contents += rf.readStringUntil('\n');
        contents += '\n';
      }
      rf.close();
      if (contents.length() > 0) {
        mqtt.publish("esp32s3/schedule/loaded", contents.c_str());
        Serial.println("Published schedule/loaded after save");
          // Also attempt to post to HTTP device_schedule endpoint if configured
          if (getDeviceScheduleUrlFromPrefs().length() > 0) {
            if (postScheduleToHttp(contents)) {
              Serial.println("Posted schedule/loaded to HTTP endpoint (on save)");
            }
          }
      }
    }
  }
}

// Pending offs
struct PendingOff { uint8_t relay; unsigned long end_ts; bool active; };
PendingOff pendingOffs[6];
void initPendingOffs() { for (int i=0;i<6;i++) pendingOffs[i].active=false; }

// WiFi reconnect helpers
const unsigned long WIFI_INIT_TIMEOUT = 20000; // ms to wait for a connect attempt
const unsigned long WIFI_RETRY_INTERVAL = 15000; // ms between automatic retries
const int WIFI_MAX_RETRIES = 6;
unsigned long lastWiFiAttempt = 0;
int wifiRetryCount = 0;
// Fallback retry behavior: after hitting WIFI_MAX_RETRIES we will continue
// attempting full connect cycles every WIFI_FALLBACK_INTERVAL until a
// total timeout WIFI_TOTAL_RETRY_TIMEOUT elapses, then we start the portal.
const unsigned long WIFI_FALLBACK_INTERVAL = 120000; // 2 minutes
const unsigned long WIFI_TOTAL_RETRY_TIMEOUT = 15UL * 60UL * 1000UL; // 15 minutes
unsigned long wifiFailureStart = 0;
bool wifiInFallback = false;
unsigned long lastWiFailureCheck = 0;

// Try to connect using stored creds (if present) or defaults. Returns true if connected.
bool tryConnectWiFiOnce() {
  // Read stored credentials (if any)
  String stored_ssid = "";
  String stored_pass = "";
  prefs.begin("wifi", true);
  stored_ssid = prefs.getString("ssid", "");
  stored_pass = prefs.getString("pass", "");
  prefs.end();

  auto attemptConnectTo = [&](const String &target_ssid, const String &target_pass) -> bool {
    // Enforce WPA2 Personal passphrase rules: empty = open network, otherwise min 8 chars
    if (target_pass.length() > 0) {
      if (!(target_pass.length() >= 8 || target_pass.length() == 64)) {
        Serial.print("Skipping connect to '"); Serial.print(target_ssid);
        Serial.println("': password too short for WPA2 Personal (min 8 chars).");
        return false;
      }
    }
    Serial.print("Scanning for '"); Serial.print(target_ssid); Serial.println("' before attempting connect...");
    int n = WiFi.scanNetworks();
    bool found = false;
    for (int i = 0; i < n; ++i) {
      if (WiFi.SSID(i) == target_ssid) { found = true; break; }
    }
    if (!found) {
      Serial.print("SSID '"); Serial.print(target_ssid);
      Serial.println("' not found in scan. Proceeding to attempt connection anyway (hotspot may be 5GHz or hidden). If problems persist, ensure hotspot is 2.4 GHz and WPA2 Personal.");
      // do not return; attempt connection anyway to support mobile hotspots and hidden SSIDs
    }
    Serial.print("Connecting to WiFi '"); Serial.print(target_ssid); Serial.println("'");
    WiFi.begin(target_ssid.c_str(), target_pass.c_str());
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_INIT_TIMEOUT) {
      Serial.print('.');
      delay(500);
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi connected, IP: "); Serial.println(WiFi.localIP());
      wifiRetryCount = 0;
      lastWiFiAttempt = millis();
      return true;
    }
    Serial.print("WiFi connect failed, status="); Serial.println(WiFi.status());
    lastWiFiAttempt = millis();
    return false;
  };

  // Prefer stored credentials first (helps when defaults are placeholders)
  String default_ssid = String(WIFI_SSID);
  String default_pass = String(WIFI_PASS);
  if (stored_ssid.length() > 0) {
    Serial.print("Attempting stored WiFi credentials: '"); Serial.print(stored_ssid); Serial.println("'...");
    if (attemptConnectTo(stored_ssid, stored_pass)) return true;
  }

  // Fall back to compiled defaults if stored credentials are missing or failed
  if (default_ssid.length() > 0) {
    Serial.print("Attempting default WiFi credentials: '"); Serial.print(default_ssid); Serial.println("'...");
    if (attemptConnectTo(default_ssid, default_pass)) return true;
  }

  // nothing worked
  wifiRetryCount++;
  return false;
}

// Forward declarations
void startConfigPortal();
void stopConfigPortal();
void startDashboardServer();
void stopDashboardServer();

// WiFi event handler: log events and attempt reconnects
void onWiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.print("Event: GOT IP -> "); Serial.println(WiFi.localIP());
    wifiRetryCount = 0;
    lastWiFiAttempt = millis();
    // start dashboard server when we have network
    if (!configPortalActive) startDashboardServer();
    // Try to fetch remote schedule (if configured) and overwrite local schedule
    if (getDeviceScheduleUrlFromPrefs().length() > 0) {
      Serial.println("Attempting to GET schedule from configured HTTP endpoint");
      if (getScheduleFromHttpAndSave()) {
        Serial.println("Fetched and saved schedule from HTTP endpoint");
      } else {
        Serial.println("No remote schedule fetched or failed");
      }
    }
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.println("Event: WIFI STA Disconnected");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("RSSI: "); Serial.println(WiFi.RSSI());
    } else {
      Serial.println("RSSI: unknown (not connected)");
    }
    if (!configPortalActive) {
      if (wifiRetryCount < WIFI_MAX_RETRIES) {
        Serial.println("Event: attempting WiFi reconnect...");
        wifiRetryCount++;
        lastWiFiAttempt = millis();
        WiFi.reconnect();
      } else {
        // Enter fallback mode: continue trying full connect cycles periodically
        if (!wifiInFallback) {
          wifiInFallback = true;
          wifiFailureStart = millis();
          Serial.println("Event: entering fallback reconnect mode (periodic attempts)");
        }
      }
    }
    // stop dashboard when disconnected
    stopDashboardServer();
  }
}

// Web handlers
// Escape string for HTML output (basic)
String escapeHTML(const String &s) {
  String out = "";
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s.charAt(i);
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c; break;
    }
  }
  return out;
}

// Format uptime given seconds into human-readable string
String formatUptime(unsigned long seconds) {
  unsigned long days = seconds / 86400UL;
  seconds %= 86400UL;
  unsigned long hours = seconds / 3600UL;
  seconds %= 3600UL;
  unsigned long minutes = seconds / 60UL;
  unsigned long secs = seconds % 60UL;
  char buf[64];
  if (days > 0) {
    // Example: "1d 02:03:04"
    snprintf(buf, sizeof(buf), "%lud %02luh %02lum %02lus", days, hours, minutes, secs);
  } else if (hours > 0) {
    // Example: "02:03:04"
    snprintf(buf, sizeof(buf), "%02luh %02lum %02lus", hours, minutes, secs);
  } else if (minutes > 0) {
    // Example: "03m 04s"
    snprintf(buf, sizeof(buf), "%02lum %02lus", minutes, secs);
  } else {
    snprintf(buf, sizeof(buf), "%luls", secs);
  }
  return String(buf);
}

void handlePortalRoot() {
  // perform a fresh scan when serving the portal so results are up-to-date
  configScanCount = WiFi.scanNetworks();
  String html = "<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head><body>";
  html += "<h3>ESP32 WiFi Setup</h3>";
  html += "<form method=\"POST\" action=\"/save\">";
  html += "SSID:<br>";
  if (configScanCount > 0) {
    html += "<select name=\"ssid\">";
    for (int i = 0; i < configScanCount; ++i) {
      String ssid = WiFi.SSID(i);
      String esc = escapeHTML(ssid);
      html += "<option value=\"" + esc + "\">" + esc + " (" + String(WiFi.RSSI(i)) + ")" + "</option>";
    }
    html += "</select><br>";
  } else {
    html += "<em>No se encontraron redes. Ingresa el SSID manualmente abajo.</em><br>";
  }
  // provide manual SSID input as fallback (also useful for hidden networks)
  html += "<br>Or enter SSID manually:<br><input name=\"ssid_manual\" type=\"text\" placeholder=\"SSID\"><br>";
  html += "Password:<br><input name=\"pass\" type=\"password\"><br><br><input type=\"submit\" value=\"Save & Connect\"></form>";
  // show currently saved device_schedule_url (if any)
  prefs.begin("cloud", true);
  String current_ds = prefs.getString("device_schedule_url", "");
  prefs.end();
  html += "<hr><h4>Cloud / Device schedule URL</h4>";
  html += "<p>Optional: endpoint where the device will GET/POST its /schedule.csv (plain CSV).</p>";
  if (current_ds.length() > 0) {
    html += "<p>Current saved URL: <strong>" + escapeHTML(current_ds) + "</strong></p>";
  } else {
    html += "<p>No device schedule URL configured.</p>";
  }
  html += "<label>Device schedule URL (http:// or https://)</label><br>";
  html += "<input name=\"device_sched_url\" type=\"text\" placeholder=\"https://myserver.example.com/api/device_schedule\" value=\"" + escapeHTML(current_ds) + "\"><br>";
  html += "<p><em>Nota:</em> Si usas un hotspot móvil, configura la zona Wi‑Fi en 2.4 GHz (no 5 GHz) y usa WPA2 Personal.</p>";
  html += "<p>Refrescar lista: <a href=\"/scan\">Scan networks</a> — también puedes usar el comando serial 'wscan'.</p>";
  html += "</body></html>";
  configServer.send(200, "text/html", html);
}

void handlePortalSave() {
  if (configServer.hasArg("ssid") || configServer.hasArg("ssid_manual")) {
    String ssid = configServer.arg("ssid");
    // prefer manual SSID if provided
    if (ssid.length() == 0 && configServer.hasArg("ssid_manual")) ssid = configServer.arg("ssid_manual");
    String pass = configServer.arg("pass");
    // Require WPA2 Personal passphrase when a password is provided
    if (pass.length() > 0 && !(pass.length() >= 8 || pass.length() == 64)) {
      String resp = "<html><body><h3>Password must be a WPA2 passphrase (min 8 chars)</h3><p>Please retry.</p></body></html>";
      configServer.send(400, "text/html", resp);
      return;
    }
    Serial.print("Portal: attempting to connect to SSID='"); Serial.print(ssid); Serial.print("' pass_len="); Serial.println(pass.length());
    // attempt connect immediately and report result to the user
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long start = millis();
    const unsigned long JOIN_TIMEOUT = 20000; // 20s
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < JOIN_TIMEOUT) {
      Serial.print('.');
      delay(500);
    }
    Serial.println();
    String resp;
    String saveMsg = "";
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Portal: connected, IP="); Serial.println(WiFi.localIP());
      // store credentials persistently now that connection succeeded
      prefs.begin("wifi", false);
      prefs.putString("ssid", ssid);
      prefs.putString("pass", pass);
      // verify readback immediately
      String verify_ssid = prefs.getString("ssid", "");
      String verify_pass = prefs.getString("pass", "");
      prefs.end();
      Serial.print("Portal: persisted ssid='"); Serial.print(verify_ssid); Serial.print("' pass_len="); Serial.println(verify_pass.length());
      resp = "<html><body><h3>Connected</h3><p>IP: " + WiFi.localIP().toString() + "</p></body></html>";
      // ensure MQTT connected if needed
      ensureMqttConnected();
      // save optional device schedule URL and validate it
      String saveMsg = "";
      if (configServer.hasArg("device_sched_url")) {
        String ds = configServer.arg("device_sched_url");
        if (ds.length() > 0) {
          // basic validation: must start with http:// or https://
          if (!(ds.startsWith("http://") || ds.startsWith("https://"))) {
            saveMsg = "<p style=\"color:orange\">Warning: URL should start with http:// or https://. Saved anyway.</p>";
          }
          // attempt a quick GET to validate reachability
          bool reachable = false;
          HTTPClient httptest;
          httptest.begin(ds);
          int code = httptest.GET();
          httptest.end();
          if (code >= 200 && code < 300) {
            reachable = true;
          }
          prefs.begin("cloud", false);
          prefs.putString("device_schedule_url", ds);
          prefs.end();
          Serial.print("Saved device_schedule_url: "); Serial.println(ds);
          if (reachable) {
            saveMsg += "<p style=\"color:green\">Device schedule URL reachable (HTTP " + String(code) + ").</p>";
          } else {
            saveMsg += "<p style=\"color:orange\">Could not reach URL (HTTP " + String(code) + "). The URL was saved but may not be valid.</p>";
          }
        } else {
          // empty value -> clear saved
          prefs.begin("cloud", false);
          prefs.remove("device_schedule_url");
          prefs.end();
          Serial.println("Cleared device_schedule_url");
          saveMsg = "<p style=\"color:blue\">Device schedule URL cleared.</p>";
        }
      }
      // stop portal now that we're connected and credentials are saved
      stopConfigPortal();
    } else {
      Serial.print("Portal: failed to connect, status="); Serial.println(WiFi.status());
      resp = "<html><body><h3>Connection failed</h3><p>Device could not connect to '" + ssid + "'. Please verify SSID/password and that the AP is 2.4 GHz and uses WPA2 Personal. Use serial 'wscan' to debug.</p></body></html>";
    }
    // append saveMsg (if any) so user sees feedback about device_schedule_url
    if (saveMsg.length() > 0) resp += saveMsg;
    configServer.send(WiFi.status() == WL_CONNECTED ? 200 : 500, "text/html", resp);
  } else {
    configServer.send(400, "text/plain", "Missing SSID");
  }
}

// Dashboard handlers and storage
void appendSample(unsigned long t, float t1, float h1, float t2, float h2) {
  sampleTimes[sampleIndex] = t;
  sample_t1[sampleIndex] = t1;
  sample_h1[sampleIndex] = h1;
  sample_t2[sampleIndex] = t2;
  sample_h2[sampleIndex] = h2;
  sampleIndex = (sampleIndex + 1) % SAMPLE_BUFFER_SIZE;
  if (sampleCount < SAMPLE_BUFFER_SIZE) sampleCount++;
}

String buildJSONSamples() {
  String out = "[";
  int start = (sampleCount == SAMPLE_BUFFER_SIZE) ? sampleIndex : 0;
  for (int i = 0; i < sampleCount; ++i) {
    int idx = (start + i) % SAMPLE_BUFFER_SIZE;
    if (i) out += ",";
    out += "{";
    out += "\"t\":" + String(sampleTimes[idx]) + ",";
    out += "\"t1\":" + String(sample_t1[idx], 2) + ",";
    out += "\"h1\":" + String(sample_h1[idx], 2) + ",";
    out += "\"t2\":" + String(sample_t2[idx], 2) + ",";
    out += "\"h2\":" + String(sample_h2[idx], 2);
    out += "}";
  }
  out += "]";
  return out;
}

void handleDashboardPage() {
  String html = "<html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">";
  html += "<title>ESP32 Dashboard</title></head><body>";
  html += "<h3>Sensor Dashboard</h3>";
  html += "<div id=\"status\">Loading...</div>";
  html += "<pre id=\"data\"></pre>";
  html += "<script>function load(){fetch('/data').then(r=>r.json()).then(d=>{document.getElementById('data').innerText=JSON.stringify(d,null,2);});}setInterval(load,5000);load();</script>";
  html += "</body></html>";
  dashboardServer.send(200, "text/html", html);
}

void handleDataJSON() {
  String j = buildJSONSamples();
  dashboardServer.send(200, "application/json", j);
}

void saveSamplesToFS() {
  File f = SPIFFS.open("/samples.jsonl", FILE_APPEND);
  if (!f) return;
  int start = (sampleCount == SAMPLE_BUFFER_SIZE) ? sampleIndex : 0;
  for (int i = 0; i < sampleCount; ++i) {
    int idx = (start + i) % SAMPLE_BUFFER_SIZE;
    char line[128];
    snprintf(line, sizeof(line), "{\"t\":%lu,\"t1\":%.2f,\"h1\":%.2f,\"t2\":%.2f,\"h2\":%.2f}\n",
             sampleTimes[idx], sample_t1[idx], sample_h1[idx], sample_t2[idx], sample_h2[idx]);
    f.print(line);
  }
  f.close();
}

// Persist a single sample to SPIFFS (JSONL line). Safe to call occasionally.
void persistSingleSampleToFS(unsigned long t, float t1, float h1, float t2, float h2) {
  if (!spiffsMountedForPersist) {
    if (!SPIFFS.begin(true)) {
      Serial.println("persistSingleSampleToFS: SPIFFS mount failed");
      return;
    }
    spiffsMountedForPersist = true;
  }
  File f = SPIFFS.open("/samples.jsonl", FILE_APPEND);
  if (!f) {
    Serial.println("persistSingleSampleToFS: failed to open /samples.jsonl");
    return;
  }
  char line[256];
  // timestamp is milliseconds since epoch on server side; here we store device millis (t)
  snprintf(line, sizeof(line), "{\"t\":%lu,\"t1\":%.2f,\"h1\":%.2f,\"t2\":%.2f,\"h2\":%.2f}\n",
           t, t1, h1, t2, h2);
  f.print(line);
  f.close();
  Serial.print("Persisted single sample to SPIFFS: "); Serial.println(line);
}

// Publish stored samples file (/samples.jsonl) over MQTT so the server
// can persist them into SQLite. This is triggered via a serial command
// (Spanish-friendly phrases supported).
void publishSamplesFileOverMQTT() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS not available");
    return;
  }
  if (!ensureMqttConnected()) {
    Serial.println("MQTT not connected; cannot publish stored samples");
    return;
  }
  const char *path = "/samples.jsonl";
  if (!SPIFFS.exists(path)) {
    Serial.println("No samples file found to publish");
    return;
  }
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) {
    Serial.println("Failed to open samples file for reading");
    return;
  }
  Serial.println("Publishing stored samples to MQTT...");
  int published = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    bool ok = mqtt.publish(MQTT_TELEMETRY_TOPIC, line.c_str());
    if (ok) published++;
    // small throttle to avoid flooding
    delay(50);
  }
  f.close();
  Serial.print("Published "); Serial.print(published); Serial.println(" samples");
  // archive or remove the file so repeated calls don't duplicate
  String archived = String(path) + ".sent";
  if (!SPIFFS.rename(path, archived.c_str())) {
    // rename may fail on some builds; fall back to delete
    SPIFFS.remove(path);
  } else {
    Serial.print("Archived samples to: "); Serial.println(archived);
  }
}

void startDashboardServer() {
  if (dashboardActive) return;
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
  } else {
    Serial.println("SPIFFS mounted");
  }
  // Protected endpoints: require token via header `X-Auth-Token` or query `?token=`
  dashboardServer.on("/dashboard", HTTP_GET, [](){
    prefs.begin("dash", true);
    String tok = prefs.getString("token", "");
    prefs.end();
    String got = dashboardServer.hasHeader("X-Auth-Token") ? dashboardServer.header("X-Auth-Token") : dashboardServer.arg("token");
    if (tok.length() > 0 && got != tok) {
      dashboardServer.send(401, "text/plain", "Unauthorized");
      return;
    }
    handleDashboardPage();
  });
  dashboardServer.on("/data", HTTP_GET, [](){
    prefs.begin("dash", true);
    String tok = prefs.getString("token", "");
    prefs.end();
    String got = dashboardServer.hasHeader("X-Auth-Token") ? dashboardServer.header("X-Auth-Token") : dashboardServer.arg("token");
    if (tok.length() > 0 && got != tok) {
      dashboardServer.send(401, "text/plain", "Unauthorized");
      return;
    }
    handleDataJSON();
  });
  dashboardServer.on("/save", HTTP_POST, [](){
    prefs.begin("dash", true);
    String tok = prefs.getString("token", "");
    prefs.end();
    String got = dashboardServer.hasHeader("X-Auth-Token") ? dashboardServer.header("X-Auth-Token") : dashboardServer.arg("token");
    if (tok.length() > 0 && got != tok) {
      dashboardServer.send(401, "text/plain", "Unauthorized");
      return;
    }
    saveSamplesToFS();
    dashboardServer.send(200, "text/plain", "saved");
  });
  // Persist a single live sample to SPIFFS via HTTP POST (protected)
  dashboardServer.on("/persist_sample", HTTP_POST, [](){
    prefs.begin("dash", true);
    String tok = prefs.getString("token", "");
    prefs.end();
    String got = dashboardServer.hasHeader("X-Auth-Token") ? dashboardServer.header("X-Auth-Token") : dashboardServer.arg("token");
    if (tok.length() > 0 && got != tok) {
      dashboardServer.send(401, "text/plain", "Unauthorized");
      return;
    }
    // Read sensors now and persist one sample
    float h1 = dht1.readHumidity();
    float t1 = dht1.readTemperature();
    float h2 = dht2.readHumidity();
    float t2 = dht2.readTemperature();
    unsigned long nowMs = millis();
    persistSingleSampleToFS(nowMs, t1, h1, t2, h2);
    String resp = "{";
    resp += "\"ts\":" + String(nowMs) + ",";
    resp += "\"t1\":" + String(t1,2) + ",\"h1\":" + String(h1,2) + ",";
    resp += "\"t2\":" + String(t2,2) + ",\"h2\":" + String(h2,2);
    resp += "}";
    dashboardServer.send(200, "application/json", resp);
  });
  dashboardServer.begin();
  dashboardActive = true;
  Serial.println("Dashboard server started at /dashboard");
}

void stopDashboardServer() {
  if (!dashboardActive) return;
  dashboardServer.stop();
  dashboardActive = false;
  Serial.println("Dashboard server stopped");
}

void startConfigPortal() {
  if (configPortalActive) return;
  Serial.println("Starting WiFi config portal (AP mode)");
  // create soft AP
  const char *apName = "ESP32-Setup";
  WiFi.disconnect(true);
  delay(100);
  // perform a STA scan first (better results), then enable AP+STA for portal
  WiFi.mode(WIFI_STA);
  delay(100);
  configScanCount = WiFi.scanNetworks();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apName);
  delay(200);
  // start web server
  configServer.on("/", handlePortalRoot);
  configServer.on("/save", HTTP_POST, handlePortalSave);
  // allow explicit scan refresh via browser
  configServer.on("/scan", HTTP_GET, [](){
    configScanCount = WiFi.scanNetworks();
    String resp = "<html><body><h3>Scan complete</h3><p>Found " + String(configScanCount) + " networks. <a href=\"/\">Back</a></p></body></html>";
    configServer.send(200, "text/html", resp);
  });
  configServer.begin();
  configPortalActive = true;
  Serial.print("Config portal started at "); Serial.println(WiFi.softAPIP());
}

void stopConfigPortal() {
  if (!configPortalActive) return;
  Serial.println("Stopping config portal");
  configServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  configPortalActive = false;
}

// Early print task: prints a short line every 200ms until stopEarlyPrint==true
void earlyPrintTask(void *pvParameters) {
  (void)pvParameters;
  while (!stopEarlyPrint) {
    Serial.println("[BOOT] initializing...");
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  vTaskDelete(NULL);
}

// Handle serial commands like "r5 1" to set relay 5 on/off
void handleSerialCommand(const String &cmd) {
  String s = cmd;
  s.trim();
  s.toLowerCase();
  if (s.length() == 0) return;
  // rN V  -> set relay N to 0/1
  if (s.charAt(0) == 'r' && s.length() > 1 && isDigit(s.charAt(1))) {
    int space = s.indexOf(' ');
    if (space <= 1) {
      Serial.println("Invalid command. Use: r<N> <0|1>");
      return;
    }
    String numStr = s.substring(1, space);
    String valStr = s.substring(space + 1);
    int relayNum = numStr.toInt();
    int val = valStr.toInt();
    if (relayNum < 1 || relayNum > 6 || (val != 0 && val != 1)) {
      Serial.println("Invalid relay number/value");
      return;
    }
    int idx = relayNum - 1;
    digitalWrite(RELAY_PINS[idx], val ? HIGH : LOW);
    int readBack = digitalRead(RELAY_PINS[idx]);
    if (ensureMqttConnected()) {
      char stateTopic[64];
      snprintf(stateTopic, sizeof(stateTopic), "esp32s3/state/relay%d", relayNum);
      mqtt.publish(stateTopic, readBack ? "1" : "0");
    }
    Serial.print("Serial: set relay "); Serial.print(relayNum);
    Serial.print(" -> "); Serial.print(val);
    Serial.print(" (pin "); Serial.print(RELAY_PINS[idx]); Serial.print(") read="); Serial.println(readBack);
    return;
  }

  // trN -> transient toggle (pulse) relay N for 500ms
  if (s.startsWith("tr") && s.length() > 2 && isDigit(s.charAt(2))) {
    String numStr = s.substring(2);
    int relayNum = numStr.toInt();
    if (relayNum < 1 || relayNum > 6) {
      Serial.println("Invalid relay number for trN");
      return;
    }
    int idx = relayNum - 1;
    int before = digitalRead(RELAY_PINS[idx]);
    digitalWrite(RELAY_PINS[idx], !before);
    Serial.print("Serial: pulse relay "); Serial.print(relayNum); Serial.println(" (on temporary)");
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(RELAY_PINS[idx], before);
    int after = digitalRead(RELAY_PINS[idx]);
    if (ensureMqttConnected()) {
      char stateTopic[64];
      snprintf(stateTopic, sizeof(stateTopic), "esp32s3/state/relay%d", relayNum);
      mqtt.publish(stateTopic, after ? "1" : "0");
    }
    Serial.print("Serial: pulse done relay "); Serial.print(relayNum);
    Serial.print(" read="); Serial.println(after);
    return;
  }

  // rs -> report relay states
  if (s == "rs") {
    Serial.println("Relay states:");
    for (int i = 0; i < 6; i++) {
      Serial.print("r"); Serial.print(i+1); Serial.print(" (pin "); Serial.print(RELAY_PINS[i]); Serial.print(") = ");
      Serial.println(digitalRead(RELAY_PINS[i]));
    }
    return;
  }

  // cycle on/off/status -> control del ciclo automático de relays
  if (s.startsWith("cycle")) {
    if (s == "cycle on") {
      autoCycle = true;
      Serial.println("Auto-cycle enabled");
      return;
    }
    if (s == "cycle off") {
      autoCycle = false;
      Serial.println("Auto-cycle disabled");
      return;
    }
    if (s == "cycle status") {
      Serial.print("Auto-cycle: "); Serial.println(autoCycle ? "ON" : "OFF");
      return;
    }
    Serial.println("Usage: cycle on|off|status");
    return;
  }

  // wscan -> scan and list WiFi networks with indices
  if (s == "wscan") {
    Serial.println("Scanning for available WiFi networks...");
    lastWiFiScanCount = WiFi.scanNetworks();
    Serial.print(lastWiFiScanCount); Serial.println(" networks found:");
    for (int i = 0; i < lastWiFiScanCount; ++i) {
      Serial.print(i);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (RSSI "); Serial.print(WiFi.RSSI(i)); Serial.print(")");
      if (WiFi.encryptionType(i) != WIFI_AUTH_OPEN) Serial.print(" *");
      Serial.println();
    }
    // do not delete scan results so the user can use wjoin
    return;
  }

  // wjoin <index> <password> -> join network from last scan with given index and password
  if (s.startsWith("wjoin ")) {
    // parse index and password (password may be empty for open networks)
    int space = s.indexOf(' ');
    String rest = s.substring(space + 1);
    rest.trim();
    int space2 = rest.indexOf(' ');
    if (space2 < 0) {
      Serial.println("Usage: wjoin <index> <password>   (use empty password for open networks)");
      return;
    }
    String idxStr = rest.substring(0, space2);
    String pass = rest.substring(space2 + 1);
    int idx = idxStr.toInt();
    if (lastWiFiScanCount <= 0) {
      Serial.println("No previous scan results. Run 'wscan' first.");
      return;
    }
    if (idx < 0 || idx >= lastWiFiScanCount) {
      Serial.println("Invalid index. Run 'wscan' to see available networks.");
      return;
    }
    String ssid = WiFi.SSID(idx);
    Serial.print("Attempting to join '"); Serial.print(ssid); Serial.println("'...");
    if (pass.length() > 0 && !(pass.length() >= 8 || pass.length() == 64)) {
      Serial.println("Refusing to join: password too short for WPA2 Personal (min 8 chars).");
      return;
    }
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long start = millis();
    const unsigned long JOIN_TIMEOUT = 20000; // 20s
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < JOIN_TIMEOUT) {
      Serial.print('.');
      delay(500);
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Connected, IP: "); Serial.println(WiFi.localIP());
      // ensure MQTT connection
      ensureMqttConnected();
    } else {
      Serial.print("Failed to connect to '"); Serial.print(ssid); Serial.println("'");
      Serial.print("WiFi status="); Serial.println(WiFi.status());
    }
    return;
  }

  // wconnect <ssid> <password> -> immediate connect to given SSID (ssid without spaces)
  if (s.startsWith("wconnect ")) {
    int space = s.indexOf(' ');
    String rest = s.substring(space + 1);
    int space2 = rest.indexOf(' ');
    if (space2 < 0) {
      Serial.println("Usage: wconnect <ssid> <password>   (ssid without spaces)");
      return;
    }
    String ssid = rest.substring(0, space2);
    String pass = rest.substring(space2 + 1);
    Serial.print("Attempting to join '"); Serial.print(ssid); Serial.println("'...");
    if (pass.length() > 0 && !(pass.length() >= 8 || pass.length() == 64)) {
      Serial.println("Refusing to join: password too short for WPA2 Personal (min 8 chars).");
      return;
    }
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long start = millis();
    const unsigned long JOIN_TIMEOUT = 20000; // 20s
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < JOIN_TIMEOUT) {
      Serial.print('.');
      delay(500);
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Connected, IP: "); Serial.println(WiFi.localIP());
      ensureMqttConnected();
    } else {
      Serial.print("Failed to connect to '"); Serial.print(ssid); Serial.println("'");
      Serial.print("WiFi status="); Serial.println(WiFi.status());
    }
    return;
  }

  // wportal -> start config portal (AP + webserver) to configure WiFi via browser
  if (s == "wportal") {
    startConfigPortal();
    Serial.println("Config portal started. Connect to WiFi 'ESP32-Setup' and open http://192.168.4.1/");
    return;
  }

  // wifi show -> display stored WiFi SSID (password not printed)
  if (s == "wifi show") {
    prefs.begin("wifi", true);
    String ssid = prefs.getString("ssid", "(none)");
    prefs.end();
    Serial.print("Stored WiFi SSID: "); Serial.println(ssid);
    return;
  }

  // wifi clear -> remove stored WiFi credentials from NVS
  if (s == "wifi clear") {
    prefs.begin("wifi", false);
    prefs.clear();
    prefs.end();
    Serial.println("Cleared stored WiFi credentials.");
    return;
  }

  // wifi set <ssid> <password>  -> save WiFi creds to NVS and attempt connect
  if (s.startsWith("wifi set ") || s.startsWith("wifi guardar ")) {
    String rest;
    if (s.startsWith("wifi set ")) rest = s.substring(strlen("wifi set "));
    else rest = s.substring(strlen("wifi guardar "));
    rest.trim();
    int space = rest.indexOf(' ');
    if (space < 1) {
      Serial.println("Usage: wifi set <ssid> <password>    (ssid without spaces)");
      return;
    }
    String ssid = rest.substring(0, space);
    String pass = rest.substring(space + 1);
    // Require WPA2 Personal passphrase when provided
    if (pass.length() > 0 && !(pass.length() >= 8 || pass.length() == 64)) {
      Serial.println("Refusing to save: password too short for WPA2 Personal (min 8 chars).");
      return;
    }
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
    Serial.print("Saved WiFi credentials for: "); Serial.println(ssid);
    // verify readback
    prefs.begin("wifi", true);
    String check_ssid = prefs.getString("ssid", "");
    prefs.end();
    Serial.print("Verified saved SSID: "); Serial.println(check_ssid);
    Serial.print("Attempting to connect to '"); Serial.print(ssid); Serial.println("'...");
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long start = millis();
    const unsigned long JOIN_TIMEOUT = 20000; // 20s
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < JOIN_TIMEOUT) {
      Serial.print('.');
      delay(500);
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Connected, IP: "); Serial.println(WiFi.localIP());
      ensureMqttConnected();
    } else {
      Serial.print("Failed to connect to '"); Serial.print(ssid); Serial.println("'");
      Serial.print("WiFi status="); Serial.println(WiFi.status());
    }
    return;
  }

  // schedule show -> print loaded schedule entries
  if (s == "schedule show") {
    Serial.print("Schedule entries (count="); Serial.print(scheduleCount); Serial.println("):");
    for (int i = 0; i < scheduleCount; ++i) {
      Serial.print(i); Serial.print(": used="); Serial.print(scheduleEvents[i].used);
      Serial.print(" ts="); Serial.print(scheduleEvents[i].ts);
      Serial.print(" epoch_ms="); Serial.print(scheduleEvents[i].epoch_ms);
      Serial.print(" relay="); Serial.print(scheduleEvents[i].relay);
      Serial.print(" dur="); Serial.println(scheduleEvents[i].duration);
    }
    // pending offs
    Serial.println("Pending offs:");
    for (int p = 0; p < 6; ++p) {
      Serial.print(p); Serial.print(": active="); Serial.print(pendingOffs[p].active);
      Serial.print(" relay="); Serial.print(pendingOffs[p].relay);
      Serial.print(" end_ts="); Serial.println(pendingOffs[p].end_ts);
    }
    return;
  }

  // Adafruit IO removed — 'aio' serial commands deprecated
  if (s.startsWith("aio ")) {
    Serial.println("Adafruit IO integration removed from firmware. 'aio' commands are deprecated.");
    return;
  }

  // dashboard token <token> -> save token for dashboard auth
  if (s.startsWith("dashboard token ")) {
    String tok = s.substring(strlen("dashboard token "));
    tok.trim();
    if (tok.length() == 0) {
      Serial.println("Usage: dashboard token <token>|show|clear");
      return;
    }
    prefs.begin("dash", false);
    prefs.putString("token", tok);
    prefs.end();
    Serial.println("Saved dashboard token.");
    return;
  }
  if (s == "dashboard token show") {
    prefs.begin("dash", true);
    String tok = prefs.getString("token", "(none)");
    prefs.end();
    Serial.print("Dashboard token: "); Serial.println(tok);
    return;
  }
  if (s == "dashboard token clear") {
    prefs.begin("dash", false);
    prefs.remove("token");
    prefs.end();
    Serial.println("Dashboard token cleared.");
    return;
  }
  // savefs -> persist in-memory samples buffer to SPIFFS
  if (s == "savefs") {
    saveSamplesToFS();
    Serial.println("Saved in-memory samples to /samples.jsonl");
    return;
  }

  // Spanish-friendly persistence command: publish SPIFFS samples to MQTT
  // so the server (which subscribes to MQTT_TELEMETRY_TOPIC) can store them
  if ((s.indexOf("persista") >= 0 || s.indexOf("persist") >= 0 || s.indexOf("persistir") >= 0 || s.indexOf("guardar") >= 0) &&
      (s.indexOf("sqlite") >= 0 || s.indexOf("sql") >= 0)) {
    Serial.println("Serial: requested persist to SQLite -> publishing stored samples...");
    publishSamplesFileOverMQTT();
    return;
  }
  Serial.print("Unknown serial command: "); Serial.println(cmd);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Serial OK");

  // Start early-print task immediately so we get output right after reset
  stopEarlyPrint = false;
  xTaskCreatePinnedToCore(earlyPrintTask, "earlyPrint", 2048, NULL, 1, &earlyPrintTaskHandle, 1);

  delay(100);

  // Early boot banner: printed immediately after Serial.begin()
  Serial.println();
  Serial.println("=== ESP32-S3 Relay-6CH Firmware ===");
  Serial.print("Build: "); Serial.print(__DATE__); Serial.print(" "); Serial.println(__TIME__);
  // Adafruit IO integration removed
  Serial.println("Starting...\n");
  // Print stored WiFi SSID at boot for diagnostics
  prefs.begin("wifi", true);
  String boot_ssid = prefs.getString("ssid", "(none)");
  prefs.end();
  Serial.print("Stored WiFi SSID (boot): "); Serial.println(boot_ssid);
  // Start frequent early prints while setup continues
  startEarlyBootPrints();

  // Configure WiFi behavior before attempting connection
  // Enable WiFi debug output to get detailed connection logs on serial
#if defined(ESP8266)
  WiFi.setDebugOutput(true);
#elif defined(ESP32)
  esp_log_level_set("wifi", ESP_LOG_INFO);
#endif
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("esp32-relay");
  WiFi.setSleep(false);

  // register WiFi event handler to follow disconnects/reconnects
  WiFi.onEvent(onWiFiEvent);

  // Start the IO connection
  // Attempt explicit WiFi connection first with timeout and diagnostics
  // Attempt connection and, on failure, start the config portal
  bool connected = tryConnectWiFiOnce();
  if (!connected) {
    Serial.println();
    // Print human-readable status
    Serial.print("WiFi connect failed, status="); Serial.print(WiFi.status());
    // Print human-readable status
    auto wifiStatusToString = [](int s)->const char* {
      switch (s) {
        case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
        case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
        case WL_CONNECTED: return "WL_CONNECTED";
        case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
        case WL_DISCONNECTED: return "WL_DISCONNECTED";
        default: return "UNKNOWN";
      }
    };
    Serial.print(" ("); Serial.print(wifiStatusToString(WiFi.status())); Serial.println(")");

    // Provide a network scan to help diagnose if the SSID is available (2.4GHz vs 5GHz)
    Serial.println("Scanning for available WiFi networks...");
    int n = WiFi.scanNetworks();
    lastWiFiScanCount = n;
    Serial.print(n); Serial.println(" networks found:");
    for (int i = 0; i < n; ++i) {
      Serial.print(i);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (RSSI "); Serial.print(WiFi.RSSI(i)); Serial.print(")");
      if (WiFi.encryptionType(i) != WIFI_AUTH_OPEN) Serial.print(" *");
      Serial.println();
    }
    // Start config portal so user can configure via WiFi if desired
    Serial.println("Starting configuration portal. Use serial 'wportal' to start later or connect to the AP.");
    startConfigPortal();
  }
  

  // Initialize DHT sensors
  dht1.begin();
  dht2.begin();
  Serial.println("DHT sensors initialized");

  // Adafruit IO removed — use MQTT topics for remote commands/state

  // Initialize relay pins and publish their initial states
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);
    delay(10);
    // publish initial state via MQTT
    if (ensureMqttConnected()) {
      char stateTopic[64];
      snprintf(stateTopic, sizeof(stateTopic), "esp32s3/state/relay%u", i+1);
      mqtt.publish(stateTopic, "0");
    }
  }

  // Initialize RGB LED (WS2812) on GPIO38 using Adafruit NeoPixel
  strip.begin();
  strip.setPixelColor(0, strip.Color(0,0,0));
  strip.show();
  // publish initial rgb via MQTT
  publishRGB();

  // If already connected to WiFi, start dashboard
  if (WiFi.status() == WL_CONNECTED && !configPortalActive) {
    startDashboardServer();
  }

  // Initialize in-memory schedule and pending offs
  initPendingOffs();
  // Try to synchronize time via NTP so epoch-based schedules work
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Attempting SNTP time sync...");
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    unsigned long tstart = millis();
    while (millis() - tstart < 5000) {
      time_t now = time(NULL);
      if (now > 1600000000) {
        haveEpoch = true;
        Serial.println("SNTP time acquired");
        break;
      }
      delay(200);
    }
    if (!haveEpoch) Serial.println("SNTP not available; epoch schedules will be ignored");
  }
  loadScheduleFromFS();

  // Signal the early-print task to stop and give it a short time to exit
  stopEarlyPrint = true;
  vTaskDelay(pdMS_TO_TICKS(300));

  // Stop early boot prints now that initialization finished
  stopEarlyBootPrints();
}

void loop() {
  // Read serial input (non-blocking) and process commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String cmd = serialLine;
      cmd.trim();
      if (cmd.length() > 0) handleSerialCommand(cmd);
      serialLine = "";
    } else {
      serialLine += c;
      if (serialLine.length() > 64) serialLine = serialLine.substring(serialLine.length() - 64);
    }
  }

  // Adafruit IO removed; skip io run loop

  // Maintain MQTT connection and loop
  if (ensureMqttConnected()) {
    mqtt.loop();
  }

  // If config portal active, handle web clients
  if (configPortalActive) {
    configServer.handleClient();
  }

  // If not connected, try periodic reconnects and fall back to config portal
  if (WiFi.status() != WL_CONNECTED && !configPortalActive) {
    unsigned long now = millis();
    // Normal retry window: attempt reconnects frequently up to WIFI_MAX_RETRIES
    if (!wifiInFallback) {
      if ((now - lastWiFiAttempt) >= WIFI_RETRY_INTERVAL) {
        if (wifiRetryCount < WIFI_MAX_RETRIES) {
          Serial.println("WiFi disconnected, attempting reconnect...");
          bool ok = tryConnectWiFiOnce();
          if (ok) {
            ensureMqttConnected();
            // reset fallback state in case it was set previously
            wifiInFallback = false;
            wifiFailureStart = 0;
          }
        } else {
          // enter fallback mode (handled in event too)
          if (!wifiInFallback) {
            wifiInFallback = true;
            wifiFailureStart = now;
            Serial.println("WiFi failed after retries; entering fallback reconnect mode.");
          }
        }
      }
    } else {
      // In fallback mode: attempt a full connect cycle every WIFI_FALLBACK_INTERVAL
      if ((now - lastWiFailureCheck) >= WIFI_FALLBACK_INTERVAL) {
        Serial.println("Fallback: attempting full WiFi connect cycle...");
        lastWiFailureCheck = now;
        bool ok = tryConnectWiFiOnce();
        if (ok) {
          Serial.println("Fallback: reconnected successfully");
          ensureMqttConnected();
          wifiInFallback = false;
          wifiFailureStart = 0;
          wifiRetryCount = 0;
        } else {
          Serial.println("Fallback: connect attempt failed");
          // if we've exceeded the total timeout, open the config portal
          if (now - wifiFailureStart >= WIFI_TOTAL_RETRY_TIMEOUT) {
            Serial.println("Fallback: total retry timeout reached; starting config portal");
            startConfigPortal();
          }
        }
      }
    }
  }

  // Visible indicators for physical verification
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  const unsigned long BLINK_INTERVAL = 500;
  if (millis() - lastBlink > BLINK_INTERVAL) {
    lastBlink = millis();
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState ? HIGH : LOW);
  }

  // Cycle through all 6 relays automatically for physical verification
  if (autoCycle) {
    static unsigned long lastCycleMillis = 0;
    static int cycleIndex = 0;
    static int cycleState = 0; // 0=off-wait, 1=on-wait
    const unsigned long CYCLE_OFF_INTERVAL = 4000; // time between activations
    const unsigned long CYCLE_ON_DURATION = 1000; // how long each relay stays on
    unsigned long nowMillis = millis();
    if (cycleState == 0) {
      if (nowMillis - lastCycleMillis >= CYCLE_OFF_INTERVAL) {
        // turn on current relay
        int idx = cycleIndex;
        digitalWrite(RELAY_PINS[idx], HIGH);
        if (ensureMqttConnected()) {
          char stateTopic[64];
          snprintf(stateTopic, sizeof(stateTopic), "esp32s3/state/relay%d", idx+1);
          mqtt.publish(stateTopic, digitalRead(RELAY_PINS[idx]) ? "1" : "0");
        }
        Serial.print("Cycle: relay on "); Serial.println(idx+1);
        cycleState = 1;
        lastCycleMillis = nowMillis;
      }
    } else {
      if (nowMillis - lastCycleMillis >= CYCLE_ON_DURATION) {
        // turn off current relay and advance
        int idx = cycleIndex;
        digitalWrite(RELAY_PINS[idx], LOW);
        if (ensureMqttConnected()) {
          char stateTopic[64];
          snprintf(stateTopic, sizeof(stateTopic), "esp32s3/state/relay%d", idx+1);
          mqtt.publish(stateTopic, digitalRead(RELAY_PINS[idx]) ? "1" : "0");
        }
        Serial.print("Cycle: relay off "); Serial.println(idx+1);
        cycleIndex = (cycleIndex + 1) % 6;
        cycleState = 0;
        lastCycleMillis = nowMillis;
      }
    }
  }

  // Example: publish a random value every 10s
  static unsigned long last = 0;
    if (millis() - last > 10000) {
    last = millis();
    int value = random(0, 2);
    Serial.print("Publishing: ");
    Serial.println(value);
    // Publish via MQTT (telemetry)
    // (Adafruit IO removed)
    // Publish via MQTT as well (telemetry placeholder)
    if (ensureMqttConnected()) {
      char buf[128];
      snprintf(buf, sizeof(buf), "{\"uptime\":%lu,\"rand\":%d}", millis()/1000, value);
      mqtt.publish(MQTT_TELEMETRY_TOPIC, buf);
    }
  }

  // Read DHT sensors and print status every DHT_INTERVAL
  if (millis() - lastDHT > DHT_INTERVAL) {
    lastDHT = millis();
    Serial.println("--- Status & DHT readings ---");
    // Board status
    unsigned long up_s = millis() / 1000;
    Serial.print("Uptime: "); Serial.println(formatUptime(up_s));
    Serial.print("Free heap: "); Serial.println(ESP.getFreeHeap());
    Serial.print("WiFi IP: ");
    if (WiFi.status() == WL_CONNECTED) Serial.println(WiFi.localIP()); else Serial.println("not connected");

    // Sensor 1
    float h1 = dht1.readHumidity();
    float t1 = dht1.readTemperature();
    if (isnan(h1) || isnan(t1)) {
      Serial.print("Sensor DHT1 (pin "); Serial.print(DHTPIN1); Serial.println(") read failed");
    } else {
      Serial.print("DHT1 ("); Serial.print(DHTPIN1); Serial.print(") - Temp: "); Serial.print(t1); Serial.print(" *C, Humidity: "); Serial.print(h1); Serial.println(" %");
    }

    // Sensor 2
    float h2 = dht2.readHumidity();
    float t2 = dht2.readTemperature();
    if (isnan(h2) || isnan(t2)) {
      Serial.print("Sensor DHT2 (pin "); Serial.print(DHTPIN2); Serial.println(") read failed");
    } else {
      Serial.print("DHT2 ("); Serial.print(DHTPIN2); Serial.print(") - Temp: "); Serial.print(t2); Serial.print(" *C, Humidity: "); Serial.print(h2); Serial.println(" %");
    }
    // Publish DHT readings via MQTT
    if (ensureMqttConnected()) {
      char payload[128];
      snprintf(payload, sizeof(payload), "{\"t1\":%.2f,\"h1\":%.2f,\"t2\":%.2f,\"h2\":%.2f}", t1, h1, t2, h2);
      mqtt.publish(MQTT_TELEMETRY_TOPIC, payload);
    }
    // Append to in-memory circular buffer
    unsigned long nowMs = millis();
    appendSample(nowMs, t1, h1, t2, h2);
    // Persist one sample every PERSIST_INTERVAL_MS to SPIFFS for server retrieval
    if ((millis() - lastPersist) >= PERSIST_INTERVAL_MS) {
      persistSingleSampleToFS(nowMs, t1, h1, t2, h2);
      lastPersist = millis();
    }
    // Print relay states as part of status block so serial shows current relay states
    Serial.println("Relay states:");
    for (int i = 0; i < 6; ++i) {
      Serial.print("r"); Serial.print(i+1);
      Serial.print(" (pin "); Serial.print(RELAY_PINS[i]); Serial.print(") = ");
      Serial.println(digitalRead(RELAY_PINS[i]));
    }
    Serial.println("-----------------------------");
  }
  // Schedule execution: check for due events and pending off timers
  unsigned long now = millis();
  // If we don't yet have epoch time, attempt a quick SNTP check occasionally
  static unsigned long lastEpochCheck = 0;
  if (!haveEpoch && (millis() - lastEpochCheck) > 5000) {
    lastEpochCheck = millis();
    time_t nowt = time(NULL);
    if (nowt > 1600000000) {
      haveEpoch = true;
      Serial.println("NTP time now available; will convert epoch-based schedules");
      // convert any stored epoch_ms entries to device-relative ts
      unsigned long long nowEpochMs = getCurrentEpochMs();
      // define a reasonable maximum future window for conversion (30 days)
      const unsigned long MAX_FUTURE_MS = 30UL * 24UL * 3600UL * 1000UL;
      for (int i = 0; i < scheduleCount; ++i) {
        if (!scheduleEvents[i].used) continue;
        if (scheduleEvents[i].epoch_ms > 0ULL && scheduleEvents[i].ts == 0) {
          if (scheduleEvents[i].epoch_ms <= nowEpochMs) {
            Serial.print("Dropping past schedule epoch_ms: "); Serial.println(scheduleEvents[i].epoch_ms);
            scheduleEvents[i].used = false;
          } else {
            unsigned long long delta64 = scheduleEvents[i].epoch_ms - nowEpochMs;
            if (delta64 > (unsigned long long)MAX_FUTURE_MS) {
              Serial.print("Dropping schedule too far in future (ms): "); Serial.println((unsigned long long)delta64);
              scheduleEvents[i].used = false;
            } else {
              unsigned long delta = (unsigned long) delta64;
              unsigned long nowMsRel = millis();
              // avoid overflow: if delta would overflow unsigned long, drop
              scheduleEvents[i].ts = nowMsRel + delta;
              scheduleEvents[i].epoch_ms = 0ULL;
              Serial.print("Converted epoch schedule to ts in ms: "); Serial.println(scheduleEvents[i].ts);
            }
          }
        }
      }
    }
  }
  // Process pending offs
  for (int i = 0; i < 6; ++i) {
    if (pendingOffs[i].active && now >= pendingOffs[i].end_ts) {
      int pin = RELAY_PINS[pendingOffs[i].relay - 1];
      digitalWrite(pin, LOW);
      if (ensureMqttConnected()) {
        char stateTopic[64];
        snprintf(stateTopic, sizeof(stateTopic), "esp32s3/state/relay%d", pendingOffs[i].relay);
        mqtt.publish(stateTopic, "0");
      }
      Serial.print("Scheduled off: relay "); Serial.print(pendingOffs[i].relay); Serial.println(" -> OFF");
      pendingOffs[i].active = false;
    }
  }
  // Process schedule events (one-shot)
  bool scheduleModified = false;
  for (int i = 0; i < scheduleCount; ++i) {
    if (!scheduleEvents[i].used) continue;
    if (now >= scheduleEvents[i].ts) {
      // Trigger relay
      uint8_t relay = scheduleEvents[i].relay;
      if (relay >= 1 && relay <= 6) {
        int pin = RELAY_PINS[relay - 1];
        digitalWrite(pin, HIGH);
        Serial.print("Scheduled on: relay "); Serial.print(relay); Serial.print(" duration_ms="); Serial.println(scheduleEvents[i].duration);
        if (ensureMqttConnected()) {
          char stateTopic[64];
          snprintf(stateTopic, sizeof(stateTopic), "esp32s3/state/relay%d", relay);
          mqtt.publish(stateTopic, "1");
        }
        // set pending off
        for (int p = 0; p < 6; ++p) {
          if (!pendingOffs[p].active) {
            pendingOffs[p].relay = relay;
            pendingOffs[p].end_ts = now + scheduleEvents[i].duration;
            pendingOffs[p].active = true;
            break;
          }
        }
      }
      // handle repeating every 8 days: reschedule by adding 8 days to ts
      if (scheduleEvents[i].repeat8) {
        const unsigned long EIGHT_DAYS_MS = 8UL * 24UL * 3600UL * 1000UL;
        // avoid overflow by computing relative
        scheduleEvents[i].ts = now + EIGHT_DAYS_MS;
        scheduleModified = true;
      } else {
        // mark event as used/consumed
        scheduleEvents[i].used = false;
        scheduleModified = true;
      }
    }
  }
  if (scheduleModified) {
    saveScheduleToFS();
  }
  // Heartbeat: print alive every HEARTBEAT_INTERVAL
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    lastHeartbeat = millis();
    Serial.print("[HEARTBEAT] alive, uptime(s): ");
    Serial.println(millis() / 1000);
  }
}