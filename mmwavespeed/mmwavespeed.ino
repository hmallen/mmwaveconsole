/*
  RD-03D Single Target Speed Tracking on ESP32-S3
  
  Author: Simplified from mmwaveconsole.ino
  Board: Heltec WiFi-LoRa 32 V3 (ESP32-S3)
  Sensor: Ai-Thinker RD-03D mmWave Radar
  Baud: 256000
  Frame size: 30 bytes (AA FF header, 8 byte payload * 3 targets, 55 CC footer)
  
  This script only tracks a single target and outputs its speed to the serial monitor.
  No GUI, web server, or multi-target functionality included.
  
  Wiring:
    RD-03D TX → ESP32(GPIO18) == Serial1 RX (RADAR_RX)
    RD-03D RX ← ESP32(GPIO17) == Serial1 TX (RADAR_TX)
    RD-03D VCC → 3V3 or 5V
    Shared GND
*/

#include <Arduino.h>

// Hardware configuration
static constexpr uint32_t BAUD_RATE = 256000UL;
static constexpr uint8_t RADAR_RX = 18;  // RX pin on ESP32 → TX of radar
static constexpr uint8_t RADAR_TX = 17;  // TX pin on ESP32 → RX of radar

// Single target command for RD-03D
static const uint8_t CMD_SINGLE[12] PROGMEM = {
  0xFD, 0xFC, 0xFB, 0xFA,
  0x02, 0x00, 0x80, 0x00,
  0x04, 0x03, 0x02, 0x01
};

// Frame parser for 30-byte frames
struct FrameParser {
  uint8_t buf[30];
  uint8_t idx = 0;
  bool syncing = true;
  unsigned long frameStartTime = 0;
  static const unsigned long FRAME_TIMEOUT_MS = 100;
  uint32_t droppedFrames = 0;
  uint32_t validFrames = 0;

  void reset() {
    idx = 0;
    syncing = true;
    frameStartTime = 0;
  }

  bool isTimedOut() {
    return (frameStartTime > 0 && (millis() - frameStartTime) > FRAME_TIMEOUT_MS);
  }

  void startFrame() {
    frameStartTime = millis();
  }
};

FrameParser parser;
HardwareSerial RadarSerial(1);  // Serial1 on ESP32-S3

// Output control - limit to 10Hz
unsigned long lastOutputTime = 0;
static const unsigned long OUTPUT_INTERVAL_MS = 100;

// Enhanced frame validation
bool validateFrame(const uint8_t f[30]) {
  // Check header
  if (f[0] != 0xAA || f[1] != 0xFF) {
    return false;
  }

  // Check footer
  if (f[28] != 0x55 || f[29] != 0xCC) {
    return false;
  }

  // Basic target count validation (should be 1-3)
  uint8_t targetCount = f[2];
  if (targetCount == 0 || targetCount > 3) {
    return false;
  }

  return true;
}

bool targetActive = false;

void parseFrame(const uint8_t f[30]) {
  if (!validateFrame(f)) {
    parser.droppedFrames++;
    return;
  }

  parser.validFrames++;

  // Rate limit output to 10Hz
  if (millis() - lastOutputTime < OUTPUT_INTERVAL_MS) {
    return;
  }

  lastOutputTime = millis();

  uint8_t num = f[2];  // Number of targets in frame

  // Process only the first active target
  for (uint8_t i = 0; i < num; i++) {
    const uint8_t base = 4 + i * 8;

    // Bounds check
    if (base + 7 >= 30) {
      break;
    }

    // Extract raw values
    uint16_t rawX = uint16_t(f[base]) | uint16_t(f[base + 1]) << 8;
    uint16_t rawY = uint16_t(f[base + 2]) | uint16_t(f[base + 3]) << 8;
    uint16_t rawS = uint16_t(f[base + 4]) | uint16_t(f[base + 5]) << 8;

    // Convert coordinates (X axis uses 0x0200 as center, Y axis uses 0x8000 baseline)
    int16_t x = int16_t(rawX - 0x0200);
    int16_t y = int16_t(rawY - 0x8000);

    // Convert speed (RD-03D uses 32768 as zero baseline)
    int16_t speed = (rawS < 0x8000) ? int16_t(rawS) : int16_t(0x8000 - rawS);

    // Calculate distance to determine if this is an active target
    float dist_cm = sqrt(double(x * x + y * y)) * 0.1;  // Convert mm to cm

    // Check if this is an active target (minimum distance or speed threshold)
    bool isActiveTarget = (dist_cm > 5.0) && (abs(speed) > 5);

    if (isActiveTarget) {
      targetActive = true;
      // Convert speed from cm/s to mph (1 cm/s = 0.0223694 mph)
      float speedMph = speed * 0.0223694;
      Serial.println(speedMph, 2);  // Output with 2 decimal places
      return;  // Only process the first active target
    } else if (targetActive == true) {
      targetActive = false;
      Serial.println(0.00, 2);
    }
  }

  // If no active targets found, output 0
  //Serial.println(0);
}

void setup() {
  Serial.begin(115200);
  RadarSerial.begin(BAUD_RATE, SERIAL_8N1, RADAR_RX, RADAR_TX);

  delay(100);

  // Send single target command to RD-03D
  uint8_t cmd_buffer[12];
  memcpy_P(cmd_buffer, CMD_SINGLE, sizeof(CMD_SINGLE));
  RadarSerial.write(cmd_buffer, sizeof(CMD_SINGLE));

  Serial.println("RD-03D Single Target Speed Tracking Started");
  Serial.println("Output: Speed values in miles per hour");

  parser.reset();
}

void loop() {
  // Handle frame timeout
  if (parser.isTimedOut()) {
    parser.reset();
    parser.droppedFrames++;
  }

  // Process incoming radar data
  while (RadarSerial.available()) {
    uint8_t b = RadarSerial.read();

    if (parser.syncing) {
      // Look for frame header: AA FF
      if (parser.idx == 0 && b == 0xAA) {
        parser.buf[parser.idx++] = b;
        parser.startFrame();
      } else if (parser.idx == 1 && b == 0xFF) {
        parser.buf[parser.idx++] = b;
        parser.syncing = false;
      } else {
        parser.reset();
      }
    } else {
      // Collect frame data
      parser.buf[parser.idx++] = b;

      if (parser.idx >= 30) {
        // Complete frame received
        parseFrame(parser.buf);
        parser.reset();
      }
    }
  }

  // Small delay to prevent overwhelming the CPU
  delay(1);
}