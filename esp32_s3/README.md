ESP32-S3 telemetry example
===========================

Copy `include/secrets.example.h` to `include/secrets.h` and fill with your WiFi credentials, `TELEMETRY_URL` and `TELEMETRY_KEY`.

Build and upload with PlatformIO:

```bash
cd esp32_s3
platformio run -e esp32s3 -t upload
platformio device monitor -e esp32s3
```

Notes:
Additional notes:
- You can provide `TELEMETRY_CA` in `secrets.h` (PEM) to enable full TLS validation; leave empty to use insecure mode (not recommended).
- The firmware uses LittleFS to persist telemetry if network fails. Queued items are stored in `/queue.txt` and flushed on reconnect.
- A simple status endpoint is available on the device at `http://<device-ip>/status` which returns JSON with `wifi`, `ip`, `queue_count` and `last_send_ms`.

## Exposing the device to the public internet (Direct port-forward)

**Warning:** exposing an IoT device directly to the internet carries risk. Prefer MQTT relay or VPN tunnels for production.

1. Set a strong username/password: edit `src/main.cpp` and change `WEB_USER`/`WEB_PASS` before deploying.

2. Upload firmware and LittleFS contents:

```bash
cd esp32_s3
platformio run -e esp32s3 -t upload
platformio run -e esp32s3 -t buildfs
platformio run -e esp32s3 -t uploadfs
```

3. Configure your router to forward an external TCP port (e.g. 8080) to the ESP's local IP on port 80.

4. (Optional) Configure Dynamic DNS to map a hostname to your home IP.

5. Access the device at `http://your-ddns:8080/` and authenticate with the credentials set in step 1.

Troubleshooting:

- If you can't reach the device from the WAN, test with a mobile network (disable Wi‑Fi on your phone) to avoid NAT loopback issues.
- Verify the ESP IP printed on the serial console and the router's DHCP mapping.

Alternatives:

- Use an MQTT relay or reverse tunnel (ngrok / Cloudflare Tunnel) to avoid exposing port-forwarded devices.
