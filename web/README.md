ESP32-S3 Telemetry Web

This is a minimal Node.js web app that subscribes to an MQTT broker and forwards messages to web clients via Socket.IO.

Quick start:

1. Install dependencies:

```bash
cd web
npm install
```

2. Run the server (default broker: test.mosquitto.org):

```bash
# optional: set MQTT_BROKER env var, e.g. mqtt://your-broker:1883
export MQTT_BROKER="mqtt://test.mosquitto.org:1883"
npm start
```

3. Open your browser at `http://localhost:3000`.

Notes:
- The server subscribes to `esp32s3/telemetry` and `esp32s3/state/#` by default.
- If you run the ESP32 firmware from this project it publishes telemetry to `esp32s3/telemetry` and relay states to `esp32s3/state/relayN`.
- For production use, use an authenticated broker and TLS (MQTTS). Set `MQTT_BROKER` accordingly.
