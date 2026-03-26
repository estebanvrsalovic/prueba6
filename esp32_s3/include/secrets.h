// Auto-generated secrets.h for local testing
#ifndef SECRETS_H
#define SECRETS_H

// Reuse local credentials if present (root include/credentials.h)
#include "../../include/credentials.h"

// Force using IPv4 public test broker IP to avoid IPv6 resolution issues
#define MQTT_BROKER_HOST "54.36.178.49"
#define MQTT_BROKER_PORT 1883

// Telemetry endpoint and key (keep defaults or override as needed)
#define TELEMETRY_URL "https://prueba6-vert.vercel.app/api/telemetry"
#define TELEMETRY_KEY ""
#define TELEMETRY_CA ""

#endif // SECRETS_H
