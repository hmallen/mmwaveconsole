#pragma once

// ===============================
// User Editable Configuration
// ===============================
// This header collects all settings that a user might want to tweak
// without digging through the main firmware source.
// Simply adjust the values below and re-flash the ESP32.

// ---- Serial & Radar ----
static constexpr uint32_t CONFIG_BAUD_RATE       = 256000UL; // UART baud rate between ESP32 and radar
static constexpr uint8_t  CONFIG_RADAR_RX_PIN    = 18;       // ESP32 pin connected to RD-03D TX
static constexpr uint8_t  CONFIG_RADAR_TX_PIN    = 17;       // ESP32 pin connected to RD-03D RX (optional)

// ---- Radar Behaviour ----
static constexpr bool     CONFIG_MULTI_TARGET    = false;    // Enable RD-03D multi-target frame type
static constexpr uint8_t  CONFIG_MAX_TARGETS     = 3;        // Max targets to track when multi-target enabled
static constexpr bool     CONFIG_ENABLE_FILTERING = false;   // Enable moving-average filter

// ---- Output / Performance ----
static constexpr uint16_t CONFIG_OUTPUT_INTERVAL_MS   = 100; // Minimum time (ms) between serial/WebSocket updates
static constexpr uint16_t CONFIG_MAX_FRAMES_PER_CYCLE = 5;   // Parse at most N frames per loop cycle

// ---- Wi-Fi / Web Server ----
static constexpr bool     CONFIG_ENABLE_WEB_SERVER    = true;  // Expose HTTP/JSON interface
static constexpr uint16_t CONFIG_WEB_SERVER_PORT      = 80;    // HTTP port
static constexpr int      CONFIG_WIFI_CONNECTION_TIMEOUT = 15000; // ms before giving up connecting
static constexpr bool     CONFIG_ENABLE_WIFI_RECONNECT   = true;  // Auto-reconnect when dropped

// ---- Data Logging ----
static constexpr bool     CONFIG_ENABLE_LOGGING     = false; // Enable SPIFFS log file
static constexpr uint16_t CONFIG_LOG_INTERVAL_MS    = 5000;  // ms between log entries
static constexpr uint32_t CONFIG_MAX_LOG_FILE_SIZE  = 1024 * 100; // 100 KB max log size

// NOTE: Wi-Fi credentials are still defined in `wifi_credentials.h` to keep them
// out of source control. Ensure `wifi_credentials.h` exists with:
//   #define WIFI_SSID     "your_ssid"
//   #define WIFI_PASSWORD "your_password"
