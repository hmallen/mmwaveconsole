/*
  RD‑03D Multi‑Target Radar Parsing on ESP32‑S3 (Heltec V3)

  Author: ChatGPT — Raw‑frame Option 3 Implementation
  Board: Heltec WiFi‑LoRa 32 V3 (ESP32‑S3)
  Sensor: Ai‑Thinker RD‑03D mmWave Radar
  Baud: 256000
  Frame size: 30 bytes (AA FF header, 8 byte payload * 3 targets, 55 CC footer)

  Wiring example:
    RD‑03D TX → ESP32(GPIO32) == Serial1 RX (RADAR_RX)
    RD‑03D RX ← ESP32(GPIO33) == Serial1 TX (RADAR_TX)
    RD‑03D VCC → 3V3 or 5V (as available)
    Shared GND
*/

//#define MULTI_TARGET

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <FS.h>
#include "wifi_credentials.h"  // WiFi credentials in separate file

static constexpr uint32_t BAUD_RATE = 256000UL;
static constexpr uint8_t RADAR_RX = 18;  // RX pin on ESP32 → TX of radar
static constexpr uint8_t RADAR_TX = 17;  // TX pin on ESP32 → RX of radar (optional)

// Store command arrays in flash memory to save RAM
static const uint8_t CMD_MULTI[12] PROGMEM = {
  0xFD, 0xFC, 0xFB, 0xFA,
  0x02, 0x00, 0x90, 0x00,
  0x04, 0x03, 0x02, 0x01
};

static const uint8_t CMD_SINGLE[12] PROGMEM = {
  0xFD, 0xFC, 0xFB, 0xFA,
  0x02, 0x00, 0x80, 0x00,
  0x04, 0x03, 0x02, 0x01
};

// Performance and output control
struct PerformanceConfig {
  unsigned long lastOutputTime = 0;
  static const unsigned long OUTPUT_INTERVAL_MS = 100; // Limit output to 10Hz
  static const uint16_t MAX_SERIAL_BUFFER = 512; // Prevent serial buffer overflow
  uint16_t processedFramesThisCycle = 0;
  static const uint16_t MAX_FRAMES_PER_CYCLE = 5; // Prevent processing backlog
};

PerformanceConfig perfConfig;

// Comprehensive configuration management
struct RadarConfig {
  // Target detection settings
  bool multiTarget = false;
  uint8_t maxTargets = 3;
  bool enableFiltering = false;
  
  // Output control
  uint16_t outputIntervalMs = 100;  // 10Hz default
  bool enableVerboseOutput = true;
  bool enableStatistics = true;
  uint16_t statisticsIntervalMs = 10000; // 10 seconds
  
  // Performance settings
  uint16_t maxFramesPerCycle = 5;
  uint16_t frameTimeoutMs = 100;
  
  // Validation settings
  bool enableChecksumValidation = true;
  bool enableBoundsChecking = true;
  uint8_t errorReportingRate = 10; // Report every Nth error
  
  // Distance and angle settings
  float distanceScale = 0.1f;  // Convert mm to cm
  bool enableAngleCalculation = true;
  
  void printConfig() {
    Serial.println(F("=== Radar Configuration ==="));
    Serial.printf("Multi-target mode: %s\n", multiTarget ? "ON" : "OFF");
    Serial.printf("Max targets: %d\n", maxTargets);
    Serial.printf("Output interval: %d ms\n", outputIntervalMs);
    Serial.printf("Frame timeout: %d ms\n", frameTimeoutMs);
    Serial.printf("Filtering: %s\n", enableFiltering ? "ON" : "OFF");
    Serial.printf("Verbose output: %s\n", enableVerboseOutput ? "ON" : "OFF");
    Serial.printf("Statistics: %s\n", enableStatistics ? "ON" : "OFF");
    Serial.println(F("============================="));
  }
  
  void setMultiTarget(bool enable) {
    multiTarget = enable;
    Serial.printf("Multi-target mode: %s\n", enable ? "ENABLED" : "DISABLED");
  }
  
  void setOutputRate(uint16_t intervalMs) {
    if (intervalMs >= 50 && intervalMs <= 5000) {
      outputIntervalMs = intervalMs;
      Serial.printf("Output interval set to %d ms\n", intervalMs);
    } else {
      Serial.println(F("Error: Output interval must be 50-5000 ms"));
    }
  }
};

RadarConfig config;

// Data filtering and smoothing
struct TargetFilter {
  static const uint8_t FILTER_SIZE = 5;
  
  // Circular buffers for moving average
  float x_history[FILTER_SIZE];
  float y_history[FILTER_SIZE];
  float speed_history[FILTER_SIZE];
  uint8_t index = 0;
  uint8_t count = 0;
  unsigned long lastUpdate = 0;
  bool isActive = false;
  
  void reset() {
    count = 0;
    index = 0;
    isActive = false;
    lastUpdate = 0;
    memset(x_history, 0, sizeof(x_history));
    memset(y_history, 0, sizeof(y_history));
    memset(speed_history, 0, sizeof(speed_history));
  }
  
  void addSample(float x, float y, float speed) {
    x_history[index] = x;
    y_history[index] = y;
    speed_history[index] = speed;
    
    index = (index + 1) % FILTER_SIZE;
    if (count < FILTER_SIZE) count++;
    
    lastUpdate = millis();
    isActive = true;
  }
  
  float getFilteredX() {
    if (count == 0) return 0;
    float sum = 0;
    for (uint8_t i = 0; i < count; i++) {
      sum += x_history[i];
    }
    return sum / count;
  }
  
  float getFilteredY() {
    if (count == 0) return 0;
    float sum = 0;
    for (uint8_t i = 0; i < count; i++) {
      sum += y_history[i];
    }
    return sum / count;
  }
  
  float getFilteredSpeed() {
    if (count == 0) return 0;
    float sum = 0;
    for (uint8_t i = 0; i < count; i++) {
      sum += speed_history[i];
    }
    return sum / count;
  }
  
  bool isExpired(unsigned long currentTime, unsigned long timeoutMs = 1000) {
    return isActive && (currentTime - lastUpdate) > timeoutMs;
  }
};

// Target tracking with filtering
struct TargetTracker {
  static const uint8_t MAX_TARGETS = 3;
  TargetFilter filters[MAX_TARGETS];
  
  void reset() {
    for (uint8_t i = 0; i < MAX_TARGETS; i++) {
      filters[i].reset();
    }
  }
  
  void updateTarget(uint8_t targetId, float x, float y, float speed) {
    if (targetId < MAX_TARGETS) {
      filters[targetId].addSample(x, y, speed);
    }
  }
  
  void cleanupExpiredTargets() {
    unsigned long currentTime = millis();
    for (uint8_t i = 0; i < MAX_TARGETS; i++) {
      if (filters[i].isExpired(currentTime)) {
        filters[i].reset();
      }
    }
  }
};

TargetTracker tracker;

// WiFi and Web Server
WebServer server(80);
struct WiFiConfig {
  const char* ssid = WIFI_SSID;           // From wifi_credentials.h
  const char* password = WIFI_PASSWORD;   // From wifi_credentials.h
  bool enableWebServer = true;
  uint16_t serverPort = 80;
  unsigned long lastDataUpdate = 0;
  int connectionTimeout = WIFI_CONNECTION_TIMEOUT;
  bool enableReconnect = ENABLE_WIFI_RECONNECT;
  
  void printStatus() {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("=== WiFi Status ==="));
      Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
      Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
      Serial.printf("Web Server: http://%s:%d\n", WiFi.localIP().toString().c_str(), serverPort);
      Serial.println(F("==================="));
    } else {
      Serial.println(F("WiFi not connected"));
    }
  }
};

WiFiConfig wifiConfig;

// JSON data structure for web API
struct RadarData {
  uint8_t targetCount = 0;
  struct Target {
    float x, y, distance, angle, speed;
    uint16_t gate;
    bool filtered;
  } targets[3];
  unsigned long timestamp;
  uint32_t validFrames;
  uint32_t droppedFrames;
  float successRate;
} currentRadarData;

// Data logging system
struct DataLogger {
  bool enableLogging = false;
  String logFileName = "/radar_log.txt";
  String configFileName = "/radar_config.json";
  unsigned long lastLogTime = 0;
  uint16_t logIntervalMs = 5000; // Log every 5 seconds
  uint32_t maxLogFileSize = 1024 * 100; // 100KB max
  uint16_t logEntryCount = 0;
  
  bool begin() {
    if (!SPIFFS.begin(true)) {
      Serial.println(F("SPIFFS initialization failed"));
      return false;
    }
    
    Serial.println(F("SPIFFS initialized successfully"));
    
    // Print file system info
    size_t totalBytes = SPIFFS.totalBytes();
    size_t usedBytes = SPIFFS.usedBytes();
    Serial.printf("SPIFFS: %u/%u bytes used (%.1f%%)\n", 
                  usedBytes, totalBytes, (float)usedBytes/totalBytes*100.0);
    
    // Load saved configuration if exists
    loadConfig();
    
    return true;
  }
  
  void logTargetData(const RadarData& data) {
    if (!enableLogging || (millis() - lastLogTime < logIntervalMs)) {
      return;
    }
    
    // Check file size and rotate if needed
    if (SPIFFS.exists(logFileName)) {
      File file = SPIFFS.open(logFileName, "r");
      if (file.size() > maxLogFileSize) {
        file.close();
        rotateLogFile();
      } else {
        file.close();
      }
    }
    
    File logFile = SPIFFS.open(logFileName, "a");
    if (!logFile) {
      Serial.println(F("Failed to open log file for writing"));
      return;
    }
    
    // Write timestamp and basic info
    logFile.printf("%lu,%u,%.1f,", data.timestamp, data.targetCount, data.successRate);
    
    // Write target data
    for (uint8_t i = 0; i < data.targetCount && i < 3; i++) {
      logFile.printf("%.1f,%.1f,%.1f,%.1f,%.1f,%u,%s", 
                     data.targets[i].x, data.targets[i].y, 
                     data.targets[i].distance, data.targets[i].angle,
                     data.targets[i].speed, data.targets[i].gate,
                     data.targets[i].filtered ? "F" : "R");
      if (i < data.targetCount - 1) logFile.print(";");
    }
    
    logFile.println();
    logFile.close();
    
    lastLogTime = millis();
    logEntryCount++;
    
    if (logEntryCount % 100 == 0) {
      Serial.printf("Logged %u entries to %s\n", logEntryCount, logFileName.c_str());
    }
  }
  
  void rotateLogFile() {
    String backupName = "/radar_log_backup.txt";
    
    // Remove old backup
    if (SPIFFS.exists(backupName)) {
      SPIFFS.remove(backupName);
    }
    
    // Rename current log to backup
    if (SPIFFS.exists(logFileName)) {
      SPIFFS.rename(logFileName, backupName);
      Serial.println(F("Log file rotated"));
    }
  }
  
  String getLogData(uint16_t maxLines = 50) {
    if (!SPIFFS.exists(logFileName)) {
      return "No log file found";
    }
    
    File logFile = SPIFFS.open(logFileName, "r");
    if (!logFile) {
      return "Failed to open log file";
    }
    
    String result = "";
    String line;
    uint16_t lineCount = 0;
    
    // Read last N lines (simple approach - read all and keep last N)
    std::vector<String> lines;
    while (logFile.available() && lineCount < 1000) { // Limit to prevent memory issues
      line = logFile.readStringUntil('\n');
      if (line.length() > 0) {
        lines.push_back(line);
        lineCount++;
      }
    }
    
    logFile.close();
    
    // Return last maxLines
    uint16_t startIndex = lines.size() > maxLines ? lines.size() - maxLines : 0;
    for (uint16_t i = startIndex; i < lines.size(); i++) {
      result += lines[i] + "\n";
    }
    
    return result;
  }
  
  void saveConfig() {
    DynamicJsonDocument doc(512);
    doc["enableLogging"] = enableLogging;
    doc["logIntervalMs"] = logIntervalMs;
    doc["maxLogFileSize"] = maxLogFileSize;
    
    // Save radar config too
    doc["multiTarget"] = config.multiTarget;
    doc["enableFiltering"] = config.enableFiltering;
    doc["outputIntervalMs"] = config.outputIntervalMs;
    doc["enableVerboseOutput"] = config.enableVerboseOutput;
    
    File configFile = SPIFFS.open(configFileName, "w");
    if (configFile) {
      serializeJson(doc, configFile);
      configFile.close();
      Serial.println(F("Configuration saved to SPIFFS"));
    } else {
      Serial.println(F("Failed to save configuration"));
    }
  }
  
  void loadConfig() {
    if (!SPIFFS.exists(configFileName)) {
      Serial.println(F("No saved configuration found"));
      return;
    }
    
    File configFile = SPIFFS.open(configFileName, "r");
    if (!configFile) {
      Serial.println(F("Failed to open configuration file"));
      return;
    }
    
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close();
    
    if (error) {
      Serial.printf("Failed to parse configuration: %s\n", error.c_str());
      return;
    }
    
    // Load logging settings
    if (doc.containsKey("enableLogging")) {
      enableLogging = doc["enableLogging"];
    }
    if (doc.containsKey("logIntervalMs")) {
      logIntervalMs = doc["logIntervalMs"];
    }
    if (doc.containsKey("maxLogFileSize")) {
      maxLogFileSize = doc["maxLogFileSize"];
    }
    
    // Load radar settings
    if (doc.containsKey("enableFiltering")) {
      config.enableFiltering = doc["enableFiltering"];
    }
    if (doc.containsKey("outputIntervalMs")) {
      config.outputIntervalMs = doc["outputIntervalMs"];
    }
    if (doc.containsKey("enableVerboseOutput")) {
      config.enableVerboseOutput = doc["enableVerboseOutput"];
    }
    
    Serial.println(F("Configuration loaded from SPIFFS"));
  }
  
  void clearLogs() {
    if (SPIFFS.exists(logFileName)) {
      SPIFFS.remove(logFileName);
    }
    if (SPIFFS.exists("/radar_log_backup.txt")) {
      SPIFFS.remove("/radar_log_backup.txt");
    }
    logEntryCount = 0;
    Serial.println(F("Log files cleared"));
  }
  
  void printStatus() {
    Serial.println(F("=== Data Logger Status ==="));
    Serial.printf("Logging: %s\n", enableLogging ? "ENABLED" : "DISABLED");
    Serial.printf("Log interval: %u ms\n", logIntervalMs);
    Serial.printf("Entries logged: %u\n", logEntryCount);
    Serial.printf("Max file size: %u bytes\n", maxLogFileSize);
    
    if (SPIFFS.exists(logFileName)) {
      File logFile = SPIFFS.open(logFileName, "r");
      Serial.printf("Current log size: %u bytes\n", logFile.size());
      logFile.close();
    } else {
      Serial.println(F("No log file exists"));
    }
    Serial.println(F("========================="));
  }
};

DataLogger dataLogger;

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
HardwareSerial RadarSerial(1);  // Serial1 on ESP32‑S3

// Enhanced frame validation with checksum
bool validateFrame(const uint8_t f[30]) {
  // Header/footer validation
  if (f[28] != 0x55 || f[29] != 0xCC || (!(f[0] == 0xAA || f[0] == 0xAD)) || f[1] != 0xFF) {
    return false;
  }
  
  // Basic checksum validation (sum of payload bytes)
  uint16_t checksum = 0;
  for (uint8_t i = 2; i < 28; i++) {
    checksum += f[i];
  }
  
  // Simple validation - ensure checksum is reasonable
  if (checksum == 0) {
    return false;
  }
  
  return true;
}

// Web server handlers
void handleRoot() {
  String html = F("<!DOCTYPE html><html><head>");
  html += F("<title>ESP32 RD-03D Radar Dashboard</title>");
  html += F("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  html += F("<link href='https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css' rel='stylesheet'>");
  html += F("<style>");
  
  // Modern CSS styling
  html += F("*{margin:0;padding:0;box-sizing:border-box}");
  html += F("body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;color:#333}");
  html += F(".dashboard{max-width:1400px;margin:0 auto;padding:20px}");
  html += F(".header{text-align:center;color:white;margin-bottom:30px}");
  html += F(".header h1{font-size:2.5em;margin-bottom:10px;text-shadow:2px 2px 4px rgba(0,0,0,0.3)}");
  html += F(".header .subtitle{font-size:1.2em;opacity:0.9}");
  
  // Grid layout
  html += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:20px;margin-bottom:30px}");
  html += F(".card{background:rgba(255,255,255,0.95);border-radius:15px;padding:20px;box-shadow:0 8px 32px rgba(0,0,0,0.1);backdrop-filter:blur(10px)}");
  
  // Radar visualization
  html += F(".radar-container{grid-column:span 2;min-height:400px}");
  html += F("#radarCanvas{width:100%;height:400px;border:2px solid #ddd;border-radius:10px;background:#1a1a2e}");
  
  // Status cards
  html += F(".stat-card{text-align:center;transition:transform 0.3s ease}");
  html += F(".stat-card:hover{transform:translateY(-5px)}");
  html += F(".stat-value{font-size:2.5em;font-weight:bold;color:#2196f3;margin:10px 0}");
  html += F(".stat-label{font-size:1.1em;color:#666;text-transform:uppercase;letter-spacing:1px}");
  html += F(".stat-icon{font-size:2em;color:#2196f3;margin-bottom:10px}");
  
  // Controls
  html += F(".controls{display:flex;flex-wrap:wrap;gap:10px;margin:20px 0}");
  html += F(".btn{padding:12px 24px;border:none;border-radius:25px;cursor:pointer;font-weight:bold;transition:all 0.3s ease;text-decoration:none;display:inline-flex;align-items:center;gap:8px}");
  html += F(".btn-primary{background:linear-gradient(45deg,#2196f3,#21cbf3);color:white}");
  html += F(".btn-secondary{background:linear-gradient(45deg,#ff6b6b,#ffa726);color:white}");
  html += F(".btn-success{background:linear-gradient(45deg,#4caf50,#8bc34a);color:white}");
  html += F(".btn:hover{transform:translateY(-2px);box-shadow:0 5px 15px rgba(0,0,0,0.2)}");
  
  // Target list
  html += F(".target-list{max-height:300px;overflow-y:auto}");
  html += F(".target-item{background:#f8f9fa;margin:10px 0;padding:15px;border-radius:10px;border-left:4px solid #2196f3;transition:all 0.3s ease}");
  html += F(".target-item:hover{background:#e3f2fd;transform:translateX(5px)}");
  
  // Toggle switches
  html += F(".toggle{position:relative;display:inline-block;width:60px;height:34px}");
  html += F(".toggle input{opacity:0;width:0;height:0}");
  html += F(".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#ccc;transition:0.4s;border-radius:34px}");
  html += F(".slider:before{position:absolute;content:'';height:26px;width:26px;left:4px;bottom:4px;background:white;transition:0.4s;border-radius:50%}");
  html += F("input:checked+.slider{background:#2196f3}");
  html += F("input:checked+.slider:before{transform:translateX(26px)}");
  
  // Responsive design
  html += F("@media (max-width:768px){.radar-container{grid-column:span 1}.grid{grid-template-columns:1fr}.controls{justify-content:center}}");
  html += F("</style>");
  
  html += F("</head><body>");
  html += F("<div class='dashboard'>");
  
  // Header
  html += F("<div class='header'>");
  html += F("<h1><i class='fas fa-radar'></i> RD-03D Radar Dashboard</h1>");
  html += F("<div class='subtitle'>Real-time mmWave Target Tracking & Analysis</div>");
  html += F("</div>");
  
  // Main grid
  html += F("<div class='grid'>");
  
  // Radar visualization
  html += F("<div class='card radar-container'>");
  html += F("<h3><i class='fas fa-crosshairs'></i> Radar Plot</h3>");
  html += F("<canvas id='radarCanvas'></canvas>");
  html += F("</div>");
  
  // Status cards
  html += F("<div class='card stat-card'>");
  html += F("<div class='stat-icon'><i class='fas fa-bullseye'></i></div>");
  html += F("<div class='stat-value' id='targetCount'>0</div>");
  html += F("<div class='stat-label'>Active Targets</div>");
  html += F("</div>");
  
  html += F("<div class='card stat-card'>");
  html += F("<div class='stat-icon'><i class='fas fa-tachometer-alt'></i></div>");
  html += F("<div class='stat-value' id='successRate'>0%</div>");
  html += F("<div class='stat-label'>Success Rate</div>");
  html += F("</div>");
  
  html += F("<div class='card stat-card'>");
  html += F("<div class='stat-icon'><i class='fas fa-signal'></i></div>");
  html += F("<div class='stat-value' id='frameCount'>0</div>");
  html += F("<div class='stat-label'>Total Frames</div>");
  html += F("</div>");
  
  // Controls panel
  html += F("<div class='card'>");
  html += F("<h3><i class='fas fa-cogs'></i> Controls</h3>");
  html += F("<div class='controls'>");
  html += F("<button class='btn btn-primary' onclick='refreshData()'><i class='fas fa-sync'></i> Refresh</button>");
  html += F("<a href='/api/data' class='btn btn-secondary'><i class='fas fa-code'></i> JSON Data</a>");
  html += F("<a href='/api/logs' class='btn btn-success'><i class='fas fa-file-alt'></i> View Logs</a>");
  html += F("</div>");
  
  // Configuration toggles
  html += F("<div style='margin-top:20px'>");
  html += F("<label style='display:flex;align-items:center;margin:10px 0'>");
  html += F("<span style='margin-right:15px'>Data Logging:</span>");
  html += F("<label class='toggle'><input type='checkbox' id='loggingToggle'><span class='slider'></span></label>");
  html += F("</label>");
  html += F("<label style='display:flex;align-items:center;margin:10px 0'>");
  html += F("<span style='margin-right:15px'>Data Filtering:</span>");
  html += F("<label class='toggle'><input type='checkbox' id='filteringToggle'><span class='slider'></span></label>");
  html += F("</label>");
  html += F("</div>");
  html += F("</div>");
  
  // Target list
  html += F("<div class='card'>");
  html += F("<h3><i class='fas fa-list'></i> Target Details</h3>");
  html += F("<div id='targetList' class='target-list'>No targets detected</div>");
  html += F("</div>");
  
  // System status panel
  html += F("<div class='card'>");
  html += F("<h3><i class='fas fa-microchip'></i> System Status</h3>");
  html += F("<div style='display:grid;grid-template-columns:1fr 1fr;gap:15px;margin-top:15px'>");
  html += F("<div><strong>Free Memory:</strong><br><span id='freeHeap'>Loading...</span> bytes</div>");
  html += F("<div><strong>WiFi Signal:</strong><br><span id='wifiRSSI'>Loading...</span> dBm</div>");
  html += F("<div><strong>Uptime:</strong><br><span id='uptime'>Loading...</span></div>");
  html += F("<div><strong>Local IP:</strong><br><span id='localIP'>Loading...</span></div>");
  html += F("</div>");
  html += F("</div>");
  
  html += F("</div>"); // Close grid
  html += F("</div>"); // Close dashboard
  
  // JavaScript functionality
  html += F("<script>");
  
  // Global variables
  html += F("let radarData = {targets: [], stats: {}};");
  html += F("let canvas, ctx;");
  html += F("let animationId;");
  
  // Initialize dashboard
  html += F("window.onload = function() {");
  html += F("  canvas = document.getElementById('radarCanvas');");
  html += F("  ctx = canvas.getContext('2d');");

  html += F("  initializeRadar();");
  html += F("  refreshData();");
  html += F("  setInterval(refreshData, 1000);");
  html += F("};");
  
  // Fetch and update data
  html += F("function refreshData() {");
  html += F("  fetch('/api/data')");
  html += F("    .then(response => response.json())");
  html += F("    .then(data => {");
  html += F("      radarData = data;");
  html += F("      updateDisplay(data);");
  html += F("      if (Array.isArray(radarData.targets)) drawRadar();");
  html += F("    })");
  html += F("    .catch(error => console.error('Error:', error));");
  html += F("}");


  // Draw radar visualization
  html += F("function drawRadar() {");
  html += F("  const centerX = canvas.width / 2;");
  html += F("  const centerY = canvas.height - 50;");
  html += F("  const maxRange = 300;");
  html += F("  const scale = (canvas.height - 100) / maxRange;");
  html += F("  canvas.width = canvas.offsetWidth || 400;");
  html += F("  canvas.height = canvas.offsetHeight || 400;");
  html += F("  ctx.fillStyle = '#1a1a2e';");
  html += F("  ctx.fillRect(0, 0, canvas.width, canvas.height);");
  html += F("  ctx.strokeStyle = '#333';");
  html += F("  ctx.lineWidth = 1;");
  html += F("  for (let r = 50; r <= maxRange; r += 50) {");
  html += F("    ctx.beginPath();");
  html += F("    ctx.arc(centerX, centerY, r * scale, Math.PI, 0);");
  html += F("    ctx.stroke();");
  html += F("  }");
  html += F("  for (let angle = -60; angle <= 60; angle += 30) {");
  html += F("    const rad = (angle * Math.PI) / 180;");
  html += F("    ctx.beginPath();");
  html += F("    ctx.moveTo(centerX, centerY);");
  html += F("    ctx.lineTo(centerX + Math.sin(rad) * maxRange * scale, centerY - Math.cos(rad) * maxRange * scale);");
  html += F("    ctx.stroke();");
  html += F("  }");
  // Distance labels
  html += F("  ctx.fillStyle = '#555';");
  html += F("  ctx.font = '10px Arial';");
  html += F("  for (let r = 50; r <= maxRange; r += 50) {");
  html += F("    ctx.fillText(r + 'cm', centerX + 6, centerY - r * scale); ");
  html += F("  }");
  // Angle labels
  html += F("  [-60, -30, 0, 30, 60].forEach(aLbl => {");
  html += F("    const radLbl = (aLbl * Math.PI) / 180;");
  html += F("    const xLbl = centerX + Math.sin(radLbl) * (maxRange + 10) * scale;");
  html += F("    const yLbl = centerY - Math.cos(radLbl) * (maxRange + 10) * scale;");
  html += F("    ctx.fillText(aLbl + '°', xLbl - (aLbl < 0 ? 20 : 0), yLbl); ");
  html += F("  });");
  html += F("  if (radarData.targets) {");
  html += F("    radarData.targets.forEach((target, i) => {");
  html += F("      const distance = parseFloat(target.distance);");
  html += F("      const angle = parseFloat(target.angle);");
  html += F("      const speed = parseFloat(target.speed);");
  html += F("      if (distance > 0 && distance <= maxRange) {");
  html += F("        const rad = (angle * Math.PI) / 180;");
  html += F("        const x = centerX + Math.sin(rad) * distance * scale;");
  html += F("        const y = centerY - Math.cos(rad) * distance * scale;");
  html += F("        let color = '#00ff00';");
  html += F("        if (Math.abs(speed) > 10) color = '#ff0000';");
  html += F("        else if (Math.abs(speed) > 5) color = '#ffff00';");
  html += F("        ctx.fillStyle = color;");
  html += F("        ctx.beginPath();");
  html += F("        ctx.arc(x, y, 8, 0, 2 * Math.PI);");
  html += F("        ctx.fill();");
  html += F("        ctx.fillStyle = '#fff';");
  html += F("        ctx.font = 'bold 10px Arial';");
  html += F("        ctx.textAlign = 'center';");
  html += F("        ctx.fillText(i + 1, x, y + 3);");
  html += F("      }");
  html += F("    });");
  html += F("  }");
  html += F("}");
  
  // Update display elements
  html += F("function updateDisplay(data) {");
  html += F("  document.getElementById('targetCount').textContent = data.targetCount || 0;");
  html += F("  const successRate = data.stats ? Math.round((data.stats.validFrames / (data.stats.validFrames + data.stats.droppedFrames)) * 100) : 0;");
  html += F("  document.getElementById('successRate').textContent = successRate + '%';");
  html += F("  document.getElementById('frameCount').textContent = data.stats ? data.stats.validFrames : 0;");
  html += F("  updateTargetList(data.targets || []);");
  html += F("  updateSystemStatus(data.system || {});");
  html += F("  updateConfigToggles(data.config || {});");
  html += F("}");
  
  // Update target list
  html += F("function updateTargetList(targets) {");
  html += F("  const targetList = document.getElementById('targetList');");
  html += F("  if (!targets.length) {");
  html += F("    targetList.innerHTML = '<div style=\"text-align:center;color:#999;padding:20px\">No targets detected</div>';");
  html += F("    return;");
  html += F("  }");
  html += F("  let content = '';");
  html += F("  targets.forEach((target, i) => {");
  html += F("    content += '<div class=\"target-item\">';");
  html += F("    content += '<strong>Target ' + (i+1) + '</strong><br>';");
  html += F("    content += 'Distance: ' + target.distance + 'cm | Angle: ' + target.angle + '° | Speed: ' + target.speed + 'cm/s<br>';");
  html += F("    content += 'Position: X=' + target.x + 'cm, Y=' + target.y + 'cm';");
  html += F("    if (target.filtered) content += ' <span style=\"color:#2196f3\">[Filtered]</span>';");
  html += F("    content += '</div>';");
  html += F("  });");
  html += F("  targetList.innerHTML = content;");
  html += F("}");
  
  // Update system status
  html += F("function updateSystemStatus(system) {");
  html += F("  document.getElementById('freeHeap').textContent = system.freeHeap || 'N/A';");
  html += F("  document.getElementById('wifiRSSI').textContent = system.wifiRSSI || 'N/A';");
  html += F("  document.getElementById('localIP').textContent = system.localIP || 'N/A';");
  html += F("  const uptime = Math.floor(system.uptime || 0);");
  html += F("  const hours = Math.floor(uptime / 3600);");
  html += F("  const minutes = Math.floor((uptime % 3600) / 60);");
  html += F("  const seconds = uptime % 60;");
  html += F("  document.getElementById('uptime').textContent = hours + 'h ' + minutes + 'm ' + seconds + 's';");
  html += F("}");
  
  // Update configuration toggles
  html += F("function updateConfigToggles(config) {");
  html += F("  const loggingToggle = document.getElementById('loggingToggle');");
  html += F("  const filteringToggle = document.getElementById('filteringToggle');");
  html += F("  if (loggingToggle) loggingToggle.checked = config.enableLogging || false;");
  html += F("  if (filteringToggle) filteringToggle.checked = config.enableFiltering || false;");
  html += F("}");
  
  // Initialize radar canvas
  html += F("function initializeRadar() {");
  html += F("  canvas.width = canvas.offsetWidth || 400;");
  html += F("  canvas.height = canvas.offsetHeight || 400;");
  html += F("  ctx.fillStyle = '#1a1a2e';");
  html += F("  ctx.fillRect(0, 0, canvas.width, canvas.height);");
  html += F("}");
  
  // Toggle functions
  html += F("function toggleLogging() {");
  html += F("  const toggle = document.getElementById('loggingToggle');");
  html += F("  fetch('/api/config', {");
  html += F("    method: 'POST',");
  html += F("    headers: {'Content-Type': 'application/json'},");
  html += F("    body: JSON.stringify({enableLogging: toggle.checked})");
  html += F("  }).then(() => refreshData());");
  html += F("}");
  
  html += F("function toggleFiltering() {");
  html += F("  const toggle = document.getElementById('filteringToggle');");
  html += F("  fetch('/api/config', {");
  html += F("    method: 'POST',");
  html += F("    headers: {'Content-Type': 'application/json'},");
  html += F("    body: JSON.stringify({enableFiltering: toggle.checked})");
  html += F("  }).then(() => refreshData());");
  html += F("}");
  
  // Event listeners
  html += F("document.getElementById('loggingToggle').addEventListener('change', toggleLogging);");
  html += F("document.getElementById('filteringToggle').addEventListener('change', toggleFiltering);");
  
  html += F("</script>");
  html += F("</body></html>");
  
  server.send(200, "text/html", html);
}

void handleAPI() {
  DynamicJsonDocument doc(1024);
  
  JsonArray targets = doc.createNestedArray("targets");
  for (int i = 0; i < currentRadarData.targetCount; i++) {
    JsonObject target = targets.createNestedObject();
    target["distance"] = currentRadarData.targets[i].distance;
    target["angle"] = currentRadarData.targets[i].angle;
    target["speed"] = currentRadarData.targets[i].speed;
    target["x"] = currentRadarData.targets[i].x;
    target["y"] = currentRadarData.targets[i].y;
    target["filtered"] = currentRadarData.targets[i].filtered;
  }
  
  doc["targetCount"] = currentRadarData.targetCount;
  doc["timestamp"] = currentRadarData.timestamp;
  
  // Statistics
  JsonObject stats = doc.createNestedObject("stats");
  stats["validFrames"] = currentRadarData.validFrames;
  stats["droppedFrames"] = currentRadarData.droppedFrames;
  stats["successRate"] = currentRadarData.successRate;
  
  // Configuration status
  JsonObject configStatus = doc.createNestedObject("config");
  configStatus["multiTarget"] = config.multiTarget;
  configStatus["enableFiltering"] = config.enableFiltering;
  configStatus["enableLogging"] = dataLogger.enableLogging;
  configStatus["enableStatistics"] = config.enableStatistics;
  configStatus["enableVerboseOutput"] = config.enableVerboseOutput;
  configStatus["maxTargets"] = config.maxTargets;
  configStatus["outputIntervalMs"] = config.outputIntervalMs;
  
  // System status
  JsonObject system = doc.createNestedObject("system");
  system["freeHeap"] = ESP.getFreeHeap();
  system["wifiRSSI"] = WiFi.RSSI();
  system["wifiConnected"] = (WiFi.status() == WL_CONNECTED);
  system["localIP"] = WiFi.localIP().toString();
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleConfig() {
  if (server.method() == HTTP_GET) {
    DynamicJsonDocument doc(512);
    doc["multiTarget"] = config.multiTarget;
    doc["maxTargets"] = config.maxTargets;
    doc["enableFiltering"] = config.enableFiltering;
    doc["outputIntervalMs"] = config.outputIntervalMs;
    doc["enableVerboseOutput"] = config.enableVerboseOutput;
    doc["enableStatistics"] = config.enableStatistics;
    doc["enableLogging"] = dataLogger.enableLogging;
    doc["logIntervalMs"] = dataLogger.logIntervalMs;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  } else if (server.method() == HTTP_POST) {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, server.arg("plain"));
    
    bool configChanged = false;
    
    if (doc.containsKey("enableFiltering")) {
      config.enableFiltering = doc["enableFiltering"];
      configChanged = true;
    }
    if (doc.containsKey("outputIntervalMs")) {
      config.setOutputRate(doc["outputIntervalMs"]);
      configChanged = true;
    }
    if (doc.containsKey("enableVerboseOutput")) {
      config.enableVerboseOutput = doc["enableVerboseOutput"];
      configChanged = true;
    }
    if (doc.containsKey("enableLogging")) {
      dataLogger.enableLogging = doc["enableLogging"];
      configChanged = true;
    }
    if (doc.containsKey("logIntervalMs")) {
      dataLogger.logIntervalMs = doc["logIntervalMs"];
      configChanged = true;
    }
    
    if (configChanged) {
      dataLogger.saveConfig();
    }
    
    server.send(200, "application/json", "{\"status\":\"updated\"}");
  }
}

void handleLogs() {
  if (server.method() == HTTP_GET) {
    String logs = dataLogger.getLogData(100);
    server.send(200, "text/plain", logs);
  } else if (server.method() == HTTP_DELETE) {
    dataLogger.clearLogs();
    server.send(200, "application/json", "{\"status\":\"logs cleared\"}");
  }
}

void setupWiFi() {
  if (!wifiConfig.enableWebServer) return;
  
  Serial.println(F("Connecting to WiFi..."));
  Serial.printf("SSID: %s\n", wifiConfig.ssid);
  WiFi.begin(wifiConfig.ssid, wifiConfig.password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < wifiConfig.connectionTimeout) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    wifiConfig.printStatus();
    
    // Setup web server routes
    server.on("/", handleRoot);
    server.on("/api/data", handleAPI);
    server.on("/api/config", HTTP_GET, handleConfig);
    server.on("/api/config", HTTP_POST, handleConfig);
    server.on("/api/logs", HTTP_GET, handleLogs);
    server.on("/api/logs", HTTP_DELETE, handleLogs);
    
    server.begin();
    Serial.println(F("Web server started"));
  } else {
    Serial.println(F("\nWiFi connection failed - continuing without web server"));
    wifiConfig.enableWebServer = false;
  }
}

void parseFrame(const uint8_t f[30]) {
  if (!validateFrame(f)) {
    // Print validation errors based on configuration
    if (config.enableVerboseOutput && (parser.droppedFrames % config.errorReportingRate == 0)) {
      Serial.println(F("Frame dropped: validation failed."));
    }
    parser.droppedFrames++;
    return;
  }
  
  parser.validFrames++;
  
  // Rate limit output based on configuration
  if (millis() - perfConfig.lastOutputTime < config.outputIntervalMs) {
    return; // Skip output but count as valid frame
  }
  
  perfConfig.lastOutputTime = millis();

  uint8_t num = f[2];
  
  // Declare activeTargets counter for single target mode filtering
  uint8_t activeTargets = 0;
  
  // Update radar data for web API (will be updated with actual count later)
  currentRadarData.timestamp = millis();
  currentRadarData.validFrames = parser.validFrames;
  currentRadarData.droppedFrames = parser.droppedFrames;
  uint32_t totalFrames = parser.validFrames + parser.droppedFrames;
  currentRadarData.successRate = totalFrames > 0 ? (float)parser.validFrames / totalFrames * 100.0 : 0;
  
  // Log data flag (will be logged after target processing)
  static bool shouldLog = false;
  shouldLog = true;
  
  // Validate target count against configuration
  if (num > config.maxTargets) {
    if (config.enableVerboseOutput) {
      Serial.printf("Warning: Target count %d exceeds maximum %d, clamping\n", num, config.maxTargets);
    }
    num = config.maxTargets;
  }
  
  if (config.enableVerboseOutput) {
    if (config.multiTarget) {
      Serial.print(F("Targets: "));
      Serial.println(num);
    } else {
      Serial.println(F("Single Target Mode"));
    }
  }

  // Process targets and filter based on mode
  for (uint8_t i = 0; i < num; i++) {
    const uint8_t base = 4 + i * 8;
    
    // Bounds check for array access
    if (base + 7 >= 30) {
      Serial.printf("Error: Target %d data extends beyond frame boundary\n", i + 1);
      break;
    }
    uint16_t rawX = uint16_t(f[base]) | uint16_t(f[base + 1]) << 8;
    uint16_t rawY = uint16_t(f[base + 2]) | uint16_t(f[base + 3]) << 8;
    uint16_t rawS = uint16_t(f[base + 4]) | uint16_t(f[base + 5]) << 8;
    uint16_t rawD = uint16_t(f[base + 6]) | uint16_t(f[base + 7]) << 8;

    int16_t x = int16_t(rawX) - 0x0200;
    int16_t y = int16_t(rawY) - 0x8000;
    
    // Fix speed overflow: RD-03D uses 32768 as zero baseline
    // Based on debug output: rawS=32784 should be small negative speed
    int16_t spd;
    
    // Enhanced debug output to understand the encoding
    if (config.enableVerboseOutput && (rawS > 32760 && rawS < 32800)) {
      Serial.printf("[SPEED_DEBUG] rawS=%u (0x%04X)\n", rawS, rawS);
    }
    
    if (rawS <= 16) {
      // Values 0-16: Very small positive speeds or zero
      spd = rawS;
    } else if (rawS < 32768) {
      // Values 17-32767: Positive speeds
      spd = rawS - 16;
    } else {
      // Values >= 32768: Negative speeds with 32768 as zero baseline
      // rawS=32768 -> spd=0, rawS=32784 -> spd=-16, etc.
      int32_t temp = 32768 - (int32_t)rawS;
      
      // Clamp to reasonable range to prevent extreme values
      if (temp < -1000) {
        spd = -1000;
      } else if (temp > 1000) {
        spd = 1000;
      } else {
        spd = (int16_t)temp;
      }
    }
    
    // Debug output for problematic speed values
    if (config.enableVerboseOutput && (rawS > 32000 || spd > 1000 || spd < -1000)) {
      Serial.printf("[DEBUG] rawS=%u (0x%04X), final_spd=%d\n", rawS, rawS, spd);
    }

    float dist_cm = sqrt(double(x * x + y * y)) * config.distanceScale;
    
    // Apply filtering if enabled
    float filtered_x = x, filtered_y = y, filtered_speed = spd;
    if (config.enableFiltering) {
      tracker.updateTarget(i, x, y, spd);
      filtered_x = tracker.filters[i].getFilteredX();
      filtered_y = tracker.filters[i].getFilteredY();
      filtered_speed = tracker.filters[i].getFilteredSpeed();
      
      // Recalculate distance with filtered values
      dist_cm = sqrt(double(filtered_x * filtered_x + filtered_y * filtered_y)) * config.distanceScale;
    }
    
    // Check if this is an active target (not just noise)
    bool isActiveTarget = (dist_cm > 5.0) || (abs(spd) > 5); // Minimum distance or speed threshold
    
    // In single target mode, only process the first active target
    if (!config.multiTarget) {
      if (!isActiveTarget) {
        continue; // Skip inactive targets
      }
      if (activeTargets >= 1) {
        break; // In single mode, only show first active target
      }
    }
    
    // Store data for web API
    if (activeTargets < 3) {
      currentRadarData.targets[activeTargets].x = config.enableFiltering ? filtered_x : x;
      currentRadarData.targets[activeTargets].y = config.enableFiltering ? filtered_y : y;
      currentRadarData.targets[activeTargets].distance = dist_cm;
      currentRadarData.targets[activeTargets].speed = config.enableFiltering ? filtered_speed : spd;
      currentRadarData.targets[activeTargets].gate = rawD;
      currentRadarData.targets[activeTargets].filtered = config.enableFiltering;
      
      if (config.enableAngleCalculation) {
        currentRadarData.targets[activeTargets].angle = atan2(double(filtered_y), double(filtered_x)) * 180.0 / PI;
      } else {
        currentRadarData.targets[activeTargets].angle = 0;
      }
    }
    
    if (config.enableVerboseOutput) {
      if (config.enableAngleCalculation) {
        float angle = atan2(double(filtered_y), double(filtered_x)) * 180.0 / PI;
        if (config.enableFiltering) {
          Serial.printf(" P%d → X:%5.0f mm  Y:%5.0f mm | Dist=%.1f cm | Angle=%.1f° | Spd=%.1f cm/s  Gate=0x%04X [F]\n",
                        activeTargets + 1, filtered_x, filtered_y, dist_cm, angle, filtered_speed, rawD);
        } else {
          Serial.printf(" P%d → X:%5d mm  Y:%5d mm | Dist=%.1f cm | Angle=%.1f° | Spd=%d cm/s  Gate=0x%04X\n",
                        activeTargets + 1, x, y, dist_cm, angle, spd, rawD);
        }
      } else {
        if (config.enableFiltering) {
          Serial.printf(" P%d → X:%5.0f mm  Y:%5.0f mm | Dist=%.1f cm | Spd=%.1f cm/s  Gate=0x%04X [F]\n",
                        activeTargets + 1, filtered_x, filtered_y, dist_cm, filtered_speed, rawD);
        } else {
          Serial.printf(" P%d → X:%5d mm  Y:%5d mm | Dist=%.1f cm | Spd=%d cm/s  Gate=0x%04X\n",
                        activeTargets + 1, x, y, dist_cm, spd, rawD);
        }
      }
    }
    
    activeTargets++; // Increment count of active targets displayed
  }
  
  // Update the actual target count for API (only active targets)
  currentRadarData.targetCount = activeTargets;
  
  // Log the complete frame data
  if (shouldLog) {
    dataLogger.logTargetData(currentRadarData);
    shouldLog = false;
  }
}

void setup() {
  Serial.begin(115200);
  RadarSerial.begin(BAUD_RATE, SERIAL_8N1, RADAR_RX, RADAR_TX);

  delay(100);
  // Initialize configuration
#ifdef MULTI_TARGET
  config.multiTarget = true;
#endif
  
  // Send appropriate command based on configuration
  uint8_t cmd_buffer[12];
  if (config.multiTarget) {
    memcpy_P(cmd_buffer, CMD_MULTI, sizeof(CMD_MULTI));
    RadarSerial.write(cmd_buffer, sizeof(CMD_MULTI));
    Serial.println(F("RD‑03D Multi‑Target Mode Enabled"));
  } else {
    memcpy_P(cmd_buffer, CMD_SINGLE, sizeof(CMD_SINGLE));
    RadarSerial.write(cmd_buffer, sizeof(CMD_SINGLE));
    Serial.println(F("RD‑03D Single‑Target Mode Enabled"));
  }
  // Initialize data logging system
  if (dataLogger.begin()) {
    dataLogger.printStatus();
  }
  
  // Print initial configuration
  config.printConfig();
  
  // Setup WiFi and web server
  setupWiFi();
  
  // Initialize tracking system
  tracker.reset();
  
  parser.reset();
}

void loop() {
  // Check for frame timeout
  // Update frame timeout from configuration
  if (parser.frameStartTime > 0 && (millis() - parser.frameStartTime) > config.frameTimeoutMs) {
    if (config.enableVerboseOutput) {
      Serial.println(F("Frame timeout - resetting parser"));
    }
    parser.droppedFrames++;
    parser.reset();
  }
  
  // Consume available bytes with limits to prevent blocking
  perfConfig.processedFramesThisCycle = 0;
  while (RadarSerial.available() > 0 && 
         perfConfig.processedFramesThisCycle < config.maxFramesPerCycle) {
    uint8_t ch = RadarSerial.read();

    if (parser.syncing) {
      if (parser.idx == 0 && (ch == 0xAA || ch == 0xAD)) {
        parser.buf[parser.idx++] = ch;
        parser.startFrame();
      } else if (parser.idx == 1 && ch == 0xFF) {
        parser.buf[parser.idx++] = ch;
        parser.syncing = false;
      } else {
        parser.idx = 0;
        parser.frameStartTime = 0;
      }
    } else {
      parser.buf[parser.idx++] = ch;
      if (parser.idx == 30) {
        parseFrame(parser.buf);
        parser.reset();
        perfConfig.processedFramesThisCycle++;
      } else if (parser.idx > 30) {  // sanity protection
        Serial.println(F("Buffer overflow - resetting parser"));
        parser.droppedFrames++;
        parser.reset();
      }
    }
  }
  
  // Cleanup expired targets if filtering is enabled
  if (config.enableFiltering) {
    static unsigned long lastCleanupTime = 0;
    if (millis() - lastCleanupTime > 5000) { // Cleanup every 5 seconds
      tracker.cleanupExpiredTargets();
      lastCleanupTime = millis();
    }
  }
  
  // Print statistics based on configuration
  static unsigned long lastStatsTime = 0;
  if (config.enableStatistics && (millis() - lastStatsTime > config.statisticsIntervalMs)) {
    uint32_t totalFrames = parser.validFrames + parser.droppedFrames;
    if (totalFrames > 0) {
      float successRate = (float)parser.validFrames / totalFrames * 100.0;
      Serial.printf("Frame stats: Valid=%lu, Dropped=%lu, Success=%.1f%%\n", 
                    parser.validFrames, parser.droppedFrames, successRate);
    }
    lastStatsTime = millis();
  }

  // Handle web server requests
  if (wifiConfig.enableWebServer && WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }
  
  delay(0);  // yields to watchdog, keeps loop alive
}
