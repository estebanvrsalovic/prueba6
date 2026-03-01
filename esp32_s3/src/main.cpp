#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <LittleFS.h>
#include <vector>
#include <cstring>
#include <WebServer.h>
#include "secrets.h"

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

  int attempts = 0;
  while (attempts < MAX_RETRY_ATTEMPTS) {
    int code = https.POST(payload);
    String resp = https.getString();
    Serial.printf("POST %s -> HTTP %d\n", TELEMETRY_URL, code);
    if (code >= 200 && code < 300) {
      if (resp.length()) Serial.println(resp);
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
  // start simple status web server
  server.on("/status", [](){
    String body = "{";
    body += "\"wifi\":\"";
    body += (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected";
    body += "\",";
    body += "\"ip\":\"" + String(WiFi.localIP().toString()) + "\",";
    body += "\"queue_count\":" + String(getQueueCount()) + ",";
    body += "\"last_send_ms\":" + String(lastSend);
    body += "}";
    server.send(200, "application/json", body);
  });
  server.begin();
  lastSend = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }

  if (millis() - lastSend >= INTERVAL_MS) {
    sendTelemetry();
    lastSend = millis();
  }

  // handle incoming HTTP requests to the status endpoint
  server.handleClient();
  delay(100);
}
