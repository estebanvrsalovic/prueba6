// Copy this file to secrets.h and fill values before building

#ifndef SECRETS_H
#define SECRETS_H

// WiFi
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASS "YOUR_PASSWORD"

// Telemetry endpoint (production or preview URL)
// e.g. "https://prueba6-vert.vercel.app/api/telemetry"
#define TELEMETRY_URL "https://prueba6-vert.vercel.app/api/telemetry"

// Telemetry key used by backend (x-telemetry-key header)
#define TELEMETRY_KEY "YOUR_TELEMETRY_KEY"

// Optional: paste the PEM CA certificate used by your telemetry endpoint here
// If left empty, the example will use insecure TLS (NOT RECOMMENDED).
// Example:
// #define TELEMETRY_CA "-----BEGIN CERTIFICATE-----\nMIID...\n-----END CERTIFICATE-----\n"
// By default keep it empty:
#define TELEMETRY_CA ""

#endif // SECRETS_H
