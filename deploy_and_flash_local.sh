#!/usr/bin/env bash
set -euo pipefail

# deploy_and_flash_local.sh
# Usage: run this on the other computer (the one that will host the server and has the ESP32 attached)
# Requirements: Node.js + npm, PlatformIO CLI, git, access to the repo files
# The script will: install web dependencies, start the local server, prepare `secrets.h`,
# build and upload the ESP32 firmware, and open a serial monitor.

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
WEB_DIR="$ROOT_DIR/web"
ESP_DIR="$ROOT_DIR/esp32_s3"

echo "Repository root: $ROOT_DIR"

command -v node >/dev/null 2>&1 || { echo "Node.js not found. Install Node.js and npm first."; exit 1; }
command -v platformio >/dev/null 2>&1 || { echo "PlatformIO CLI not found. Install PlatformIO first."; exit 1; }

# Determine local IP to make the telemetry URL reachable from the ESP32
LOCAL_IP="$(hostname -I | awk '{print $1}')"
if [ -z "$LOCAL_IP" ]; then
  echo "Could not determine local IP. Please set LOCAL_IP environment variable to host IP.";
  exit 1
fi

echo "Detected local IP: $LOCAL_IP"

read -p "WiFi SSID for the ESP32: " WIFI_SSID
read -s -p "WiFi password for the ESP32: " WIFI_PASS
echo
read -p "Telemetry shared key to use (TELEMETRY_KEY): " TELEMETRY_KEY

TELEMETRY_URL="http://$LOCAL_IP:3000/api/telemetry"
echo "Telemetry URL that will be written to firmware: $TELEMETRY_URL"

echo "\n--- Installing web dependencies (this may take a moment) ---"
cd "$WEB_DIR"
npm ci --no-audit --no-fund

echo "Starting server (background). Logs: $WEB_DIR/server.log"
nohup npm start > "$WEB_DIR/server.log" 2>&1 &
SERVER_PID=$!
echo $SERVER_PID > "$WEB_DIR/.server.pid"
sleep 1
echo "Server started with PID $SERVER_PID"

echo "Waiting 1s and checking /history endpoint..."
sleep 1
curl -sS "http://localhost:3000/history" || echo "Failed to reach local server; check $WEB_DIR/server.log"

echo "\n--- Preparing firmware secrets ---"
if [ ! -d "$ESP_DIR" ]; then
  echo "ESP32 example directory not found at $ESP_DIR"; exit 1
fi

SECRETS_EXAMPLE="$ESP_DIR/include/secrets.example.h"
SECRETS_H="$ESP_DIR/include/secrets.h"

if [ ! -f "$SECRETS_EXAMPLE" ]; then
  echo "secrets.example.h not found: $SECRETS_EXAMPLE"; exit 1
fi

cp -f "$SECRETS_EXAMPLE" "$SECRETS_H"
# replace placeholders
sed -i "s#\(#\#define WIFI_SSID \".*\"#\#define WIFI_SSID \"$WIFI_SSID\"#" "$SECRETS_H" || true
sed -i "s#\(#\#define WIFI_PASS \".*\"#\#define WIFI_PASS \"$WIFI_PASS\"#" "$SECRETS_H" || true
sed -i "s#\(#\#define TELEMETRY_URL \".*\"#\#define TELEMETRY_URL \"$TELEMETRY_URL\"#" "$SECRETS_H" || true
sed -i "s#\(#\#define TELEMETRY_KEY \".*\"#\#define TELEMETRY_KEY \"$TELEMETRY_KEY\"#" "$SECRETS_H" || true

echo "Wrote $SECRETS_H with provided values"

echo "\n--- Building and uploading firmware via PlatformIO ---"
cd "$ESP_DIR"
platformio run --target upload

echo "Firmware upload finished. Opening serial monitor (press Ctrl-C to exit)"
platformio device monitor

echo "Script finished. Server is running at http://$LOCAL_IP:3000" 
