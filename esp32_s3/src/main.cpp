#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <LittleFS.h>
#include <DHT.h>
#include <vector>
#include <memory>
#include <cstring>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include "secrets.h"

// Access point fallback credentials (used if STA connect fails)
const char *AP_SSID = "ESP32-AP";
const char *AP_PASS = "esp32pass";

// AP fallback timeout: disable AP after this many ms with no connected clients
const unsigned long AP_TIMEOUT_MS = 4UL * 60UL * 1000UL; // 4 minutes
static unsigned long apStartMillis = 0;
static bool apActive = false;
// Periodic AP window (enable temporary AP periodically to allow local access)
const unsigned long AP_WINDOW_INTERVAL_MS = 10UL * 60UL * 1000UL; // every 10 minutes
const unsigned long AP_WINDOW_DURATION_MS = 60UL * 1000UL; // 1 minute
static unsigned long apWindowNextMillis = 0;
static unsigned long apWindowEndMillis = 0;
// WiFi reconnect backoff state
static unsigned long nextWifiAttemptMillis = 0;
static int wifiRetryCount = 0;
const unsigned long WIFI_BACKOFF_BASE_MS = 5000UL; // base 5s
const unsigned long WIFI_BACKOFF_MAX_MS = 10UL * 60UL * 1000UL; // cap 10m
// Device identifier used in MQTT topics
const char *DEVICE_ID = "esp32s3-01";

// MQTT broker defaults (can be overridden by defining MQTT_BROKER_HOST/PORT in secrets.h)
// MQTT broker host/port (use macros from secrets.h if provided)
#ifndef MQTT_BROKER_HOST
const char *mqtt_broker_host = "test.mosquitto.org";
#else
const char *mqtt_broker_host = MQTT_BROKER_HOST;
#endif
#ifndef MQTT_BROKER_PORT
const int mqtt_broker_port = 1883;
#else
const int mqtt_broker_port = MQTT_BROKER_PORT;
#endif

WiFiClient espWifiClient;
PubSubClient mqttClient(espWifiClient);

// DHT sensors (DHT22)
static const int DHT1_PIN = 15;
static const int DHT2_PIN = 16;
static const int DHT_TYPE = DHT22;
DHT dht1(DHT1_PIN, DHT_TYPE);
DHT dht2(DHT2_PIN, DHT_TYPE);
// last-known readings (used as fallback when read fails)
float last_t1 = 0.0f;
float last_h1 = 0.0f;
float last_t2 = 0.0f;
float last_h2 = 0.0f;

// Simple base64 encoding for binary file contents
static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
String base64Encode(const uint8_t *data, size_t len) {
  String out;
  out.reserve(((len + 2) / 3) * 4 + 1);
  uint32_t val = 0;
  int valb = -6;
  for (size_t i = 0; i < len; i++) {
    val = (val << 8) + data[i];
    valb += 8;
    while (valb >= 0) {
      out += b64_table[(val >> valb) & 0x3F];
      valb -= 6;
    }
  }
  if (valb > -6) out += b64_table[((val << 8) >> (valb + 8)) & 0x3F];
  while (out.length() % 4) out += '=';
  return out;
}

// Return a simple mime type by extension
const char *mimeForPath(const String &path) {
  if (path.endsWith(".html") || path.endsWith(".htm")) return "text/html";
  if (path.endsWith(".css")) return "text/css";
  if (path.endsWith(".js")) return "application/javascript";
  if (path.endsWith(".png")) return "image/png";
  if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
  if (path.endsWith(".svg")) return "image/svg+xml";
  return "application/octet-stream";
}

// Handle incoming file request MQTT messages
void handleFileRequest(const char *topic, const uint8_t *payload, unsigned int length) {
  // payload expected to be JSON: {"path":"/index.html","req_id":"abc123"}
  String p((const char *)payload, length);
  String path;
  String req_id;
  // crude parsing
  int pi = p.indexOf("\"path\"");
  if (pi >= 0) {
    int c = p.indexOf(':', pi);
    int q1 = p.indexOf('"', c);
    int q2 = p.indexOf('"', q1 + 1);
    if (q1 >= 0 && q2 > q1) path = p.substring(q1 + 1, q2);
  }
  int ri = p.indexOf("\"req_id\"");
  if (ri >= 0) {
    int c = p.indexOf(':', ri);
    int q1 = p.indexOf('"', c);
    int q2 = p.indexOf('"', q1 + 1);
    if (q1 >= 0 && q2 > q1) req_id = p.substring(q1 + 1, q2);
  }
  if (path.length() == 0 || req_id.length() == 0) {
    Serial.println("Invalid file request payload");
    return;
  }
  if (path == "/") path = "/index.html";
  Serial.printf("File request for %s (req=%s)\n", path.c_str(), req_id.c_str());
  if (!LittleFS.exists(path.c_str())) {
    // respond with error
    String topicResp = String("esp32s3/file/response/") + DEVICE_ID + "/" + req_id;
    String resp = "{\"error\":\"not_found\",\"path\":\"" + path + "\"}";
    mqttClient.publish(topicResp.c_str(), resp.c_str());
    return;
  }
  File f = LittleFS.open(path.c_str(), FILE_READ);
  if (!f) {
    String topicResp = String("esp32s3/file/response/") + DEVICE_ID + "/" + req_id;
    String resp = "{\"error\":\"open_failed\",\"path\":\"" + path + "\"}";
    mqttClient.publish(topicResp.c_str(), resp.c_str());
    return;
  }
  size_t size = f.size();
  std::unique_ptr<uint8_t[]> buf(new uint8_t[size]);
  f.read(buf.get(), size);
  f.close();
  String b64 = base64Encode(buf.get(), size);
  String topicResp = String("esp32s3/file/response/") + DEVICE_ID + "/" + req_id;
  String resp = "{";
  resp += "\"path\":\"" + path + "\",";
  resp += "\"content_type\":\"" + String(mimeForPath(path)) + "\",";
  resp += "\"content_base64\":\"" + b64 + "\"";
  resp += "}";
  // publish (may be large)
  mqttClient.publish(topicResp.c_str(), resp.c_str());
}

// MQTT callback wrapper
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t(topic);
  if (t.startsWith("esp32s3/file/request/")) {
    handleFileRequest(topic, payload, length);
    return;
  }
}

// Connect to MQTT broker and subscribe to file request topic
void mqttConnectAndSubscribe() {
  if (mqttClient.connected()) return;
  Serial.printf("Connecting to MQTT %s:%d\n", mqtt_broker_host, mqtt_broker_port);
  mqttClient.setServer(mqtt_broker_host, mqtt_broker_port);
  mqttClient.setCallback(mqttCallback);
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    String clientId = String("esp32s3-") + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("MQTT connected");
      // subscribe to file requests for this device
      String topic = String("esp32s3/file/request/") + DEVICE_ID;
      mqttClient.subscribe(topic.c_str());
      break;
    }
    attempts++;
    delay(1000);
  }
}

// Public web access credentials (change before exposing device)
const char *WEB_USER = "admin";
const char *WEB_PASS = "esp32web";

const unsigned long INTERVAL_MS = 15UL * 1000UL;
unsigned long lastSend = 0;

const char *QUEUE_PATH = "/queue.txt";
const int MAX_RETRY_ATTEMPTS = 5;
WebServer server(80);

// Append payload to persistent queue
void enqueuePayload(const String &payload) {
  File f = LittleFS.open(QUEUE_PATH, FILE_APPEND);
  if (!f) {
    Serial.println("Failed to open queue file for append");
    return;
  }
  f.println(payload);
  f.close();
}

// Try to flush queued payloads (returns number remaining)
int flushQueue() {
  if (!LittleFS.exists(QUEUE_PATH)) return 0;
  File f = LittleFS.open(QUEUE_PATH, FILE_READ);
  if (!f) return 0;
  // Read all lines, compact (remove empty), dedupe (preserve order),
  // then send as a single JSON array payload to the server.
  std::vector<String> lines;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    // keep original formatting of line (expected to be JSON)
    lines.push_back(line);
  }
  f.close();

  if (lines.empty()) {
    LittleFS.remove(QUEUE_PATH);
    return 0;
  }

  // dedupe preserving order
  std::vector<String> unique;
  for (size_t i = 0; i < lines.size(); ++i) {
    const String &ln = lines[i];
    bool seen = false;
    for (size_t j = 0; j < unique.size(); ++j) {
      if (unique[j] == ln) { seen = true; break; }
    }
    if (!seen) unique.push_back(ln);
  }

  if (unique.empty()) {
    LittleFS.remove(QUEUE_PATH);
    return 0;
  }

  // build JSON array payload
  String payload = "[";
  for (size_t i = 0; i < unique.size(); ++i) {
    payload += unique[i];
    if (i + 1 < unique.size()) payload += ",";
  }
  payload += "]";

  bool ok = false;
  int attempts = 0;
  while (attempts < MAX_RETRY_ATTEMPTS) {
    WiFiClientSecure client;
    if (strlen(TELEMETRY_CA) > 10) client.setCACert(TELEMETRY_CA);
    else client.setInsecure();
    HTTPClient https;
    if (!https.begin(client, TELEMETRY_URL)) {
      Serial.println("Failed to begin HTTP for queued batch");
      attempts++;
    } else {
      https.addHeader("Content-Type", "application/json");
      https.addHeader("x-telemetry-key", TELEMETRY_KEY);
      int code = https.POST(payload);
      String resp = https.getString();
      Serial.printf("Queued batch POST -> HTTP %d\n", code);
      if (code >= 200 && code < 300) {
        ok = true;
        https.end();
        break;
      } else {
        attempts++;
        https.end();
        long backoff = (1 << attempts) * 1000;
        delay(backoff);
      }
    }
  }

  if (ok) {
    LittleFS.remove(QUEUE_PATH);
    return 0;
  }

  // if failed, rewrite compacted queue (unique) back to file to save flash and avoid duplicates
  File out = LittleFS.open(QUEUE_PATH, FILE_WRITE);
  if (!out) {
    Serial.println("Failed to rewrite queue file");
    return unique.size();
  }
  for (auto &it : unique) out.println(it);
  out.close();
  return unique.size();
}

// Count queued items without modifying the queue
int getQueueCount() {
  if (!LittleFS.exists(QUEUE_PATH)) return 0;
  File f = LittleFS.open(QUEUE_PATH, FILE_READ);
  if (!f) return 0;
  int count = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) count++;
  }
  f.close();
  return count;
}

// Attempt to connect to WiFi once (non-blocking caller should schedule retries)
// Returns true if connected after this attempt.
bool connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.print("Connecting to WiFi (attempt)");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  // short bounded wait to give the stack a chance
  while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println("WiFi connect attempt failed");
  return false;
}

void sendTelemetry() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Not connected to WiFi, skipping telemetry");
    return;
  }
  // First try to flush any queued messages
  int remaining = flushQueue();
  if (remaining > 0) {
    Serial.printf("Queue has %d remaining items, will try later\n", remaining);
  }

  WiFiClientSecure client;
  if (strlen(TELEMETRY_CA) > 10) {
    client.setCACert(TELEMETRY_CA);
  } else {
    client.setInsecure(); // For testing only. Use proper cert verification in production.
  }

  HTTPClient https;
  if (!https.begin(client, TELEMETRY_URL)) {
    Serial.println("Failed to begin HTTP request");
    return;
  }

  https.addHeader("Content-Type", "application/json");
  https.addHeader("x-telemetry-key", TELEMETRY_KEY);

  // Read DHT sensors and update last-known values (fall back to last values on read error)
  // Attempt multiple reads for more reliable DHT values
  float t1 = NAN, h1 = NAN, t2 = NAN, h2 = NAN;
  const int DHT_READ_ATTEMPTS = 3;
  for (int a = 0; a < DHT_READ_ATTEMPTS; ++a) {
    t1 = dht1.readTemperature();
    h1 = dht1.readHumidity();
    if (!isnan(t1) && !isnan(h1)) break;
    delay(200);
  }
  if (!isnan(t1)) last_t1 = t1; else Serial.println("DHT1 read failed (NaN)");
  if (!isnan(h1)) last_h1 = h1; else Serial.println("DHT1 humidity read failed (NaN)");
  for (int a = 0; a < DHT_READ_ATTEMPTS; ++a) {
    t2 = dht2.readTemperature();
    h2 = dht2.readHumidity();
    if (!isnan(t2) && !isnan(h2)) break;
    delay(200);
  }
  if (!isnan(t2)) last_t2 = t2; else Serial.println("DHT2 read failed (NaN)");
  if (!isnan(h2)) last_h2 = h2; else Serial.println("DHT2 humidity read failed (NaN)");

  // Build JSON payload including t1/h1 (and t2/h2)
  // Include diagnostic flags indicating whether last read was valid
  bool dht1_ok = !isnan(t1) && !isnan(h1);
  bool dht2_ok = !isnan(t2) && !isnan(h2);
  String payload = "{\"device_id\":\"esp32s3-01\"";
  payload += ",\"t1\":" + String(last_t1, 2);
  payload += ",\"h1\":" + String(last_h1, 2);
  payload += ",\"t2\":" + String(last_t2, 2);
  payload += ",\"h2\":" + String(last_h2, 2);
  payload += ",\"dht1_ok\":";
  payload += (dht1_ok ? "true" : "false");
  payload += ",\"dht2_ok\":";
  payload += (dht2_ok ? "true" : "false");
  payload += "}";
  // Publish immediately to MQTT relay so server UI can receive telemetry even
  // if HTTP delivery is delayed. MQTT publish is best-effort here.
  if (mqttClient.connected()) {
    mqttClient.publish("esp32s3/telemetry", payload.c_str());
  }

  int attempts = 0;
  while (attempts < MAX_RETRY_ATTEMPTS) {
    int code = https.POST(payload);
    String resp = https.getString();
    Serial.printf("POST %s -> HTTP %d\n", TELEMETRY_URL, code);
    if (code >= 200 && code < 300) {
      if (resp.length()) Serial.println(resp);
      // Also publish telemetry over MQTT relay if available
      if (mqttClient.connected()) {
        mqttClient.publish("esp32s3/telemetry", payload.c_str());
      }
      https.end();
      return;
    }
    attempts++;
    long backoff = (1 << attempts) * 1000;
    Serial.printf("POST failed, attempt %d, backing off %ld ms\n", attempts, backoff);
    delay(backoff);
  }
  // if reached here, enqueue for later delivery
  Serial.println("Max attempts reached, enqueueing payload");
  enqueuePayload(payload);
  https.end();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  if (!LittleFS.begin()) {
    Serial.println("LittleFS init failed");
  } else {
    Serial.println("LittleFS mounted");
  }
  connectWifi();
  // If we failed to connect as STA, start an access point for direct access
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Starting Access Point fallback");
    bool ok = WiFi.softAP(AP_SSID, AP_PASS);
    if (ok) {
      Serial.print("AP started, IP: ");
      Serial.println(WiFi.softAPIP());
      // mark AP active and start timeout countdown
      apActive = true;
      apStartMillis = millis();
    } else {
      Serial.println("Failed to start AP");
    }
  }
  // Serve files from LittleFS but require HTTP Basic auth for public exposure
  auto requiresAuth = []() -> bool {
    if (!server.authenticate(WEB_USER, WEB_PASS)) {
      server.requestAuthentication();
      return true; // authentication requested
    }
    return false;
  };

  // helper to detect basic content-type by extension
  auto contentType = [](const String &path)->const char* {
    if (path.endsWith(".html") || path.endsWith(".htm")) return "text/html";
    if (path.endsWith(".css")) return "text/css";
    if (path.endsWith(".js")) return "application/javascript";
    if (path.endsWith(".png")) return "image/png";
    if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
    if (path.endsWith(".svg")) return "image/svg+xml";
    if (path.endsWith(".json")) return "application/json";
    if (path.endsWith(".txt")) return "text/plain";
    return "application/octet-stream";
  };

  // Serve status but require auth
  server.on("/status", HTTP_GET, [&](){
    if (requiresAuth()) return;
    String body = "{";
    body += "\"wifi\":\"";
    body += (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected";
    body += "\",";
    body += "\"ip\":\"" + String(WiFi.localIP().toString()) + "\",";
    body += "\"queue_count\":" + String(getQueueCount()) + ",";
    body += "\"last_send_ms\":" + String(lastSend);
    body += "}";
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", body);
  });

  // Generic file handler (requires auth)
  server.onNotFound([&](){
    if (requiresAuth()) return;
    String uri = server.uri();
    if (uri == "/") uri = "/index.html";
    if (!LittleFS.exists(uri.c_str())) {
      server.send(404, "text/plain", "Not found");
      return;
    }
    File f = LittleFS.open(uri.c_str(), FILE_READ);
    if (!f) {
      server.send(500, "text/plain", "Failed to open file");
      return;
    }
    server.streamFile(f, contentType(uri));
    f.close();
  });
  // server already configured above
  server.begin();
  lastSend = millis();

  // attempt MQTT connection
  mqttConnectAndSubscribe();

  // initialize DHT sensors
  dht1.begin();
  dht2.begin();
}

void loop() {
  // WiFi reconnect with exponential backoff
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() >= nextWifiAttemptMillis) {
      bool ok = connectWifi();
      if (ok) {
        wifiRetryCount = 0;
        nextWifiAttemptMillis = 0;
      } else {
        wifiRetryCount++;
        unsigned long backoff = WIFI_BACKOFF_BASE_MS * (1UL << min(wifiRetryCount, 10));
        if (backoff > WIFI_BACKOFF_MAX_MS) backoff = WIFI_BACKOFF_MAX_MS;
        nextWifiAttemptMillis = millis() + backoff;
        Serial.printf("Scheduling next WiFi attempt in %lu ms\n", backoff);
      }
    }
  }

  // ensure MQTT connected when on network
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) mqttConnectAndSubscribe();
    mqttClient.loop();
  }

  if (millis() - lastSend >= INTERVAL_MS) {
    sendTelemetry();
    lastSend = millis();
  }

  // handle incoming HTTP requests to the status endpoint
  server.handleClient();

  // AP periodic window: enable a temporary AP if device offline
  if (!apActive && WiFi.status() != WL_CONNECTED) {
    if (apWindowNextMillis == 0) apWindowNextMillis = millis() + AP_WINDOW_INTERVAL_MS;
    else if (millis() >= apWindowNextMillis) {
      Serial.println("Enabling periodic AP window for local access");
      bool ok = WiFi.softAP(AP_SSID, AP_PASS);
      if (ok) {
        apActive = true;
        apStartMillis = 0; // reset client activity timer
        apWindowEndMillis = millis() + AP_WINDOW_DURATION_MS;
      } else {
        Serial.println("Failed to start AP for window");
        apWindowNextMillis = millis() + AP_WINDOW_INTERVAL_MS;
      }
    }
  }

  // AP timeout: disable AP after AP_TIMEOUT_MS if no clients connected
  if (apActive) {
    int clients = WiFi.softAPgetStationNum();
    if (clients > 0) {
      // client present — cancel countdown
      apStartMillis = 0;
    } else {
      // no client connected; start countdown if not already started
      if (apStartMillis == 0) apStartMillis = millis();
      else if (millis() - apStartMillis >= AP_TIMEOUT_MS) {
        Serial.println("AP timeout reached with no clients; disabling AP");
        WiFi.softAPdisconnect(true);
        apActive = false;
        apStartMillis = 0;
        // schedule next window
        apWindowNextMillis = millis() + AP_WINDOW_INTERVAL_MS;
        apWindowEndMillis = 0;
      }
    }
    // also disable if the periodic window ended
    if (apWindowEndMillis != 0 && millis() >= apWindowEndMillis) {
      Serial.println("Periodic AP window ended; disabling AP");
      WiFi.softAPdisconnect(true);
      apActive = false;
      apWindowEndMillis = 0;
      apWindowNextMillis = millis() + AP_WINDOW_INTERVAL_MS;
    }
  }

  delay(100);
}
