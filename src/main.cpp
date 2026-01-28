#include <Arduino.h>
#include "credentials.h"
#include "AdafruitIO_WiFi.h"
#include "AdafruitIO_Feed.h"
#include <Adafruit_NeoPixel.h>

// Initialize the Adafruit IO WiFi client
AdafruitIO_WiFi io(AIO_USERNAME, AIO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *ledFeed = io.feed("led");

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
const uint8_t RELAY_PINS[6] = {1, 2, 41, 42, 45, 46};
AdafruitIO_Feed *relayFeeds[6];

// WS2812 (NeoPixel) RGB LED on GPIO38
#define RGB_PIN 38
#define NUM_RGB 1
Adafruit_NeoPixel strip(NUM_RGB, RGB_PIN, NEO_GRB + NEO_KHZ800);
AdafruitIO_Feed *rgbFeed = nullptr;
uint8_t current_r = 0, current_g = 0, current_b = 0;

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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(100);

  // Start the IO connection
  io.connect();

  // Attach a message handler to the feed
  ledFeed->onMessage(handleMessage);

  // Create and attach relay feeds
  relayFeeds[0] = io.feed("relay1"); relayFeeds[0]->onMessage(handleRelay1);
  relayFeeds[1] = io.feed("relay2"); relayFeeds[1]->onMessage(handleRelay2);
  relayFeeds[2] = io.feed("relay3"); relayFeeds[2]->onMessage(handleRelay3);
  relayFeeds[3] = io.feed("relay4"); relayFeeds[3]->onMessage(handleRelay4);
  relayFeeds[4] = io.feed("relay5"); relayFeeds[4]->onMessage(handleRelay5);
  relayFeeds[5] = io.feed("relay6"); relayFeeds[5]->onMessage(handleRelay6);

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
  // create rgb feed and attach handler
  rgbFeed = io.feed("rgb");
  rgbFeed->onMessage(handleRGB);
  // publish initial rgb
  publishRGB();
}

void loop() {
  // Required to maintain the connection to Adafruit IO
  io.run();

  // Example: publish a random value every 10s
  static unsigned long last = 0;
  if (millis() - last > 10000) {
    last = millis();
    int value = random(0, 2);
    Serial.print("Publishing: ");
    Serial.println(value);
    ledFeed->save(value);
  }
}