#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <LittleFS.h>
#include <vector>
#include <cstring>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include "secrets.h"

// Access point fallback credentials (used if STA connect fails)
const char *AP_SSID = "ESP32-AP";
const char *AP_PASS = "esp32pass";

// Device identifier used in MQTT topics
const char *DEVICE_ID = "esp32s3-01";

// MQTT broker defaults (can be overridden by defining MQTT_BROKER_HOST/PORT in secrets.h)
#ifndef MQTT_BROKER_HOST
const char *MQTT_BROKER_HOST = "test.mosquitto.org";
#else
const char *MQTT_BROKER_HOST = MQTT_BROKER_HOST;
#endif
#ifndef MQTT_BROKER_PORT
const int MQTT_BROKER_PORT = 1883;
#else
const int MQTT_BROKER_PORT = MQTT_BROKER_PORT;
#endif

WiFiClient espWifiClient;
PubSubClient mqttClient(espWifiClient);

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
  Serial.printf("Connecting to MQTT %s:%d\n", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
  mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
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

  std::vector<String> remaining;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    bool ok = false;
    // try to send with limited retries
    int attempts = 0;
    while (attempts < MAX_RETRY_ATTEMPTS) {
      if (line.length() == 0) break;
      WiFiClientSecure client;
      if (strlen(TELEMETRY_CA) > 10) {
        client.setCACert(TELEMETRY_CA);
      } else {
        client.setInsecure();
      }
      HTTPClient https;
      if (!https.begin(client, TELEMETRY_URL)) {
        Serial.println("Failed to begin HTTP for queued item");
        attempts++;
      } else {
        https.addHeader("Content-Type", "application/json");
        https.addHeader("x-telemetry-key", TELEMETRY_KEY);
        int code = https.POST(line);
        String resp = https.getString();
        Serial.printf("Queued POST -> HTTP %d\n", code);
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
    if (!ok) remaining.push_back(line);
  }
  f.close();

  // rewrite queue with remaining
  if (remaining.empty()) {
    LittleFS.remove(QUEUE_PATH);
    return 0;
  }
  File out = LittleFS.open(QUEUE_PATH, FILE_WRITE);
  if (!out) {
    Serial.println("Failed to rewrite queue file");
    return remaining.size();
  }
  for (auto &it : remaining) {
    out.println(it);
  }
  out.close();
  return remaining.size();
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

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed");
  }
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

  String payload = "{\"device_id\":\"esp32s3-01\",\"t1\":23.4,\"h1\":52.1}";
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
  server.on("/status", HTTP_GET, [](){
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
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }

  // ensure MQTT connected
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
  delay(100);
}
