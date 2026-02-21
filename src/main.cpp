#include <Arduino.h>
#include "credentials.h"
#include "AdafruitIO_WiFi.h"
#include "AdafruitIO_Feed.h"
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
// FreeRTOS for early printing task
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Initialize the Adafruit IO WiFi client pointer (constructed at runtime using stored creds)
AdafruitIO_WiFi *io = nullptr;
AdafruitIO_Feed *ledFeed = nullptr;

// Ensure LED_BUILTIN is defined for this board
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// Callback for incoming feed messages
void handleMessage(AdafruitIO_Data *data) {
  Serial.print("Received -> ");
  Serial.println(data->toString());
  int v = data->toInt();
  digitalWrite(LED_BUILTIN, v ? HIGH : LOW);
}

// Relay pin mapping (board: ESP32-S3-Relay-6CH)
// Avoid GPIO1/GPIO3 (UART0) which are used for serial TX/RX
const uint8_t RELAY_PINS[6] = {4, 5, 41, 42, 45, 46};
AdafruitIO_Feed *relayFeeds[6];

// Early boot printing task
TaskHandle_t earlyPrintTaskHandle = NULL;
volatile bool stopEarlyPrint = false;

// forward declaration for task function
void earlyPrintTask(void *pvParameters);


// WS2812 (NeoPixel) RGB LED on GPIO38
#define RGB_PIN 38
#define NUM_RGB 1
Adafruit_NeoPixel strip(NUM_RGB, RGB_PIN, NEO_GRB + NEO_KHZ800);
AdafruitIO_Feed *rgbFeed = nullptr;
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
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("MQTT in ["); Serial.print(t); Serial.print("] -> "); Serial.println(msg);
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
    return true;
  }
  Serial.print("MQTT connect failed, state="); Serial.println(mqtt.state());
  return false;
}

// Initialize Adafruit IO and feeds using stored or default credentials
void initAdafruitIO() {
  if (io != nullptr) return;
  String stored_ssid = "";
  String stored_pass = "";
  prefs.begin("wifi", true);
  stored_ssid = prefs.getString("ssid", "");
  stored_pass = prefs.getString("pass", "");
  prefs.end();

  // Allow Adafruit IO credentials to be stored in NVS under namespace "aio".
  String stored_aio_user = "";
  String stored_aio_key = "";
  prefs.begin("aio", true);
  stored_aio_user = prefs.getString("user", "");
  stored_aio_key = prefs.getString("key", "");
  prefs.end();

  const char* use_ssid = stored_ssid.length() ? stored_ssid.c_str() : WIFI_SSID;
  const char* use_pass = stored_ssid.length() ? stored_pass.c_str() : WIFI_PASS;

  const char* use_user = stored_aio_user.length() ? stored_aio_user.c_str() : AIO_USERNAME;
  const char* use_key = stored_aio_key.length() ? stored_aio_key.c_str() : AIO_KEY;

  io = new AdafruitIO_WiFi(use_user, use_key, use_ssid, use_pass);
  // create feeds
  ledFeed = io->feed("led");
  relayFeeds[0] = io->feed("relay1");
  relayFeeds[1] = io->feed("relay2");
  relayFeeds[2] = io->feed("relay3");
  relayFeeds[3] = io->feed("relay4");
  relayFeeds[4] = io->feed("relay5");
  relayFeeds[5] = io->feed("relay6");
  rgbFeed = io->feed("rgb");
}
uint8_t current_r = 0, current_g = 0, current_b = 0;

// DHT22 sensors (AM2302) on pins 15 and 16
#define DHTPIN1 15
#define DHTPIN2 16
#define DHTTYPE DHT22
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);

// DHT read interval (ms)
const unsigned long DHT_INTERVAL = 15000;
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
  if (rgbFeed) rgbFeed->save(String(buf));
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

// Handler for rgb feed
void handleRGB(AdafruitIO_Data *data) {
  String s = data->toString();
  uint8_t r, g, b;
  if (parseColorString(s, r, g, b)) {
    current_r = r; current_g = g; current_b = b;
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
    publishRGB();
    Serial.print("RGB set to "); Serial.println(s);
  } else {
    Serial.print("Invalid rgb format: "); Serial.println(s);
  }
}

// Individual handlers for each relay feed
void handleRelay1(AdafruitIO_Data *data) { digitalWrite(RELAY_PINS[0], data->toInt() ? HIGH : LOW); relayFeeds[0]->save(digitalRead(RELAY_PINS[0])); Serial.println("relay1 updated"); }
void handleRelay2(AdafruitIO_Data *data) { digitalWrite(RELAY_PINS[1], data->toInt() ? HIGH : LOW); relayFeeds[1]->save(digitalRead(RELAY_PINS[1])); Serial.println("relay2 updated"); }
void handleRelay3(AdafruitIO_Data *data) { digitalWrite(RELAY_PINS[2], data->toInt() ? HIGH : LOW); relayFeeds[2]->save(digitalRead(RELAY_PINS[2])); Serial.println("relay3 updated"); }
void handleRelay4(AdafruitIO_Data *data) { digitalWrite(RELAY_PINS[3], data->toInt() ? HIGH : LOW); relayFeeds[3]->save(digitalRead(RELAY_PINS[3])); Serial.println("relay4 updated"); }
void handleRelay5(AdafruitIO_Data *data) { digitalWrite(RELAY_PINS[4], data->toInt() ? HIGH : LOW); relayFeeds[4]->save(digitalRead(RELAY_PINS[4])); Serial.println("relay5 updated"); }
void handleRelay6(AdafruitIO_Data *data) { digitalWrite(RELAY_PINS[5], data->toInt() ? HIGH : LOW); relayFeeds[5]->save(digitalRead(RELAY_PINS[5])); Serial.println("relay6 updated"); }

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

// WiFi reconnect helpers
const unsigned long WIFI_INIT_TIMEOUT = 20000; // ms to wait for a connect attempt
const unsigned long WIFI_RETRY_INTERVAL = 15000; // ms between automatic retries
const int WIFI_MAX_RETRIES = 6;
unsigned long lastWiFiAttempt = 0;
int wifiRetryCount = 0;

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
    Serial.print("Scanning for '"); Serial.print(target_ssid); Serial.println("' before attempting connect...");
    int n = WiFi.scanNetworks();
    bool found = false;
    for (int i = 0; i < n; ++i) {
      if (WiFi.SSID(i) == target_ssid) { found = true; break; }
    }
    if (!found) {
      Serial.print("SSID '"); Serial.print(target_ssid); Serial.println("' not found in scan. Skipping connect attempt.");
      return false;
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

  // 1) Try default credentials first
  String default_ssid = String(WIFI_SSID);
  String default_pass = String(WIFI_PASS);
  if (attemptConnectTo(default_ssid, default_pass)) return true;

  // 2) If default failed and stored creds exist and differ, try stored creds
  if (stored_ssid.length() > 0 && stored_ssid != default_ssid) {
    Serial.println("Attempting stored WiFi credentials after default failed...");
    if (attemptConnectTo(stored_ssid, stored_pass)) return true;
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
        Serial.println("Event: max WiFi retries reached, starting config portal");
        startConfigPortal();
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

void handlePortalRoot() {
  String html = "<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head><body>";
  html += "<h3>ESP32 WiFi Setup</h3>";
  html += "<form method=\"POST\" action=\"/save\">SSID:<br><select name=\"ssid\">";
  for (int i = 0; i < configScanCount; ++i) {
    String ssid = WiFi.SSID(i);
    String esc = escapeHTML(ssid);
    html += "<option value=\"" + esc + "\">" + esc + " (" + String(WiFi.RSSI(i)) + ")" + "</option>";
  }
  html += "</select><br>Password:<br><input name=\"pass\" type=\"password\"><br><br><input type=\"submit\" value=\"Save & Connect\"></form>";
  html += "<p>Use serial command 'wscan' to refresh network list from serial.</p>";
  html += "</body></html>";
  configServer.send(200, "text/html", html);
}

void handlePortalSave() {
  if (configServer.hasArg("ssid")) {
    String ssid = configServer.arg("ssid");
    String pass = configServer.arg("pass");
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
    String resp = "<html><body><h3>Saved credentials. Attempting to connect...</h3><p>The device will try to connect to: " + ssid + "</p></body></html>";
    configServer.send(200, "text/html", resp);
    // attempt connect (non-blocking: stop portal and let loop attempt)
    stopConfigPortal();
    WiFi.begin(ssid.c_str(), pass.c_str());
    delay(100);
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
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apName);
  delay(200);
  // scan networks for selection
  configScanCount = WiFi.scanNetworks();
  // start web server
  configServer.on("/", handlePortalRoot);
  configServer.on("/save", HTTP_POST, handlePortalSave);
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
    if (relayFeeds[idx]) relayFeeds[idx]->save(readBack);
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
    if (relayFeeds[idx]) relayFeeds[idx]->save(after);
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
      // reconnect Adafruit IO
      if (!io) initAdafruitIO();
      if (io) io->connect();
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
      if (!io) initAdafruitIO();
      if (io) io->connect();
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

  // aio set <user> <key>  -> store Adafruit IO credentials to NVS and reconnect
  if (s.startsWith("aio set ")) {
    String rest = s.substring(8);
    int space = rest.indexOf(' ');
    if (space < 1) {
      Serial.println("Usage: aio set <username> <key>");
      return;
    }
    String user = rest.substring(0, space);
    String key = rest.substring(space + 1);
    prefs.begin("aio", false);
    prefs.putString("user", user);
    prefs.putString("key", key);
    prefs.end();
    Serial.println("Saved AIO credentials to NVS. Reinitializing Adafruit IO...");
    if (io) {
      io->wifi_disconnect();
      delete io;
      io = nullptr;
    }
    initAdafruitIO();
    if (io) io->connect();
    return;
  }

  if (s == "aio show") {
    prefs.begin("aio", true);
    String user = prefs.getString("user", "(none)");
    prefs.end();
    Serial.print("AIO user: "); Serial.println(user);
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
  Serial.print("AIO user: "); Serial.println(AIO_USERNAME);
  Serial.println("Starting...\n");
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

  // Initialize and connect to Adafruit IO (will use WiFi if available)
  initAdafruitIO();
  if (io) io->connect();
  // report Adafruit IO connection status
  if (io) {
    Serial.print("Adafruit IO status: ");
    Serial.println(io->statusText());
  }

  // Initialize DHT sensors
  dht1.begin();
  dht2.begin();
  Serial.println("DHT sensors initialized");

  // Attach a message handler to the feed
  if (ledFeed) ledFeed->onMessage(handleMessage);

  // Attach handlers to existing relay feeds (created in initAdafruitIO)
  if (relayFeeds[0]) relayFeeds[0]->onMessage(handleRelay1);
  if (relayFeeds[1]) relayFeeds[1]->onMessage(handleRelay2);
  if (relayFeeds[2]) relayFeeds[2]->onMessage(handleRelay3);
  if (relayFeeds[3]) relayFeeds[3]->onMessage(handleRelay4);
  if (relayFeeds[4]) relayFeeds[4]->onMessage(handleRelay5);
  if (relayFeeds[5]) relayFeeds[5]->onMessage(handleRelay6);

  // Initialize relay pins and publish their initial states
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);
    delay(10);
    // publish initial state
    relayFeeds[i]->save(0);
  }

  // Initialize RGB LED (WS2812) on GPIO38 using Adafruit NeoPixel
  strip.begin();
  strip.setPixelColor(0, strip.Color(0,0,0));
  strip.show();
  // attach handler to rgb feed
  if (rgbFeed) rgbFeed->onMessage(handleRGB);
  // publish initial rgb
  publishRGB();

  // If already connected to WiFi, start dashboard
  if (WiFi.status() == WL_CONNECTED && !configPortalActive) {
    startDashboardServer();
  }

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

  // Required to maintain the connection to Adafruit IO
  if (io) io->run();

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
    if ((millis() - lastWiFiAttempt) >= WIFI_RETRY_INTERVAL) {
      if (wifiRetryCount < WIFI_MAX_RETRIES) {
        Serial.println("WiFi disconnected, attempting reconnect...");
        bool ok = tryConnectWiFiOnce();
        if (ok) {
          if (!io) initAdafruitIO();
          if (io) io->connect();
        }
      } else {
        Serial.println("WiFi failed after retries; starting config portal.");
        startConfigPortal();
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
        if (relayFeeds[idx]) relayFeeds[idx]->save(digitalRead(RELAY_PINS[idx]));
        Serial.print("Cycle: relay on "); Serial.println(idx+1);
        cycleState = 1;
        lastCycleMillis = nowMillis;
      }
    } else {
      if (nowMillis - lastCycleMillis >= CYCLE_ON_DURATION) {
        // turn off current relay and advance
        int idx = cycleIndex;
        digitalWrite(RELAY_PINS[idx], LOW);
        if (relayFeeds[idx]) relayFeeds[idx]->save(digitalRead(RELAY_PINS[idx]));
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
    if (io && WiFi.status() == WL_CONNECTED) {
      if (ledFeed) ledFeed->save(value);
    } else {
      Serial.println("Publish skipped: Adafruit IO not connected");
    }
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
    Serial.print("Uptime (s): "); Serial.println(millis() / 1000);
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
    Serial.println("-----------------------------");
  }
  // Heartbeat: print alive every HEARTBEAT_INTERVAL
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    lastHeartbeat = millis();
    Serial.print("[HEARTBEAT] alive, uptime(s): ");
    Serial.println(millis() / 1000);
  }
}