# Prueba MQTT (ESP32)

Instrucciones rápidas:

- Edita `include/credentials.h` para ajustar tu SSID y contraseña WiFi.
- Las credenciales WiFi ya están puestas (SSID: `Redmi 9T`, PASS: `murcia09`).
- Compilar y subir con PlatformIO (desde VS Code):

```bash
pio run --target upload -e esp32s3usbotg
```

- Abrir monitor serie:

```bash
pio device monitor -e esp32s3usbotg
```

Qué hace el firmware:

 - Conecta a la red WiFi y publica/recibe comandos vía MQTT.
 - Publica telemetría en `esp32s3/telemetry` y escucha comandos en `esp32s3/command/#`.

 - Nota: cambia el pin `LED_BUILTIN` si tu placa usa otro pin para el LED integrado.

Relés (pines en la placa ESP32-S3-Relay-6CH):

- **Relay1:** GPIO1
- **Relay2:** GPIO2
- **Relay3:** GPIO41
- **Relay4:** GPIO42
- **Relay5:** GPIO45
- **Relay6:** GPIO46

MQTT topics:

- `relay1` .. `relay6`: enviar `0` o `1` para apagar/encender el relé correspondiente.

- `rgb`: aceptar `#RRGGBB` o `R,G,B` para establecer color del LED WS2812 conectado en GPIO38.

Nota: el firmware asume relés activos en HIGH. Si tus relés son activos en LOW, invierte las señales (`digitalWrite(..., !value)`).
