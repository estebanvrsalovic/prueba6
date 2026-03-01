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
