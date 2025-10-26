/**
 * ELEGOO Smart Robot Car V4.0 - WiFi Bridge
 *
 * ESP32 WiFi-to-Serial bridge for connecting ELEGOO car to backend server.
 * Forwards motor commands from server to Arduino via Serial.
 * Streams IMU data from Arduino to server via WebSocket.
 *
 * Hardware:
 * - ESP32 (or ESP8266)
 * - ELEGOO Smart Robot Car V4.0 (Arduino Uno + Motor Shield + MPU6050)
 *
 * Wiring:
 * - ESP32 TX2 (GPIO17) → Arduino RX (Pin 0)
 * - ESP32 RX2 (GPIO16) → Arduino TX (Pin 1)
 * - ESP32 GND → Arduino GND
 * - ESP32 5V → Arduino 5V
 */

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// ============================================================================
// CONFIGURATION - UPDATE THESE VALUES
// ============================================================================

// WiFi credentials
const char* WIFI_SSID = "YOUR_WIFI_SSID";        // Change this
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"; // Change this

// Server details
const char* SERVER_HOST = "192.168.1.100";  // Change to your server IP
const int SERVER_PORT = 8000;
const char* WEBSOCKET_PATH = "/ws/arduino";

// Serial communication with Arduino Uno
#define SERIAL_BAUD 9600
#define ARDUINO_SERIAL Serial2  // Use Serial2 for ESP32 (TX2=GPIO17, RX2=GPIO16)

// LED pin for status indication
#define STATUS_LED 2  // Built-in LED on most ESP32 boards

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

WebSocketsClient webSocket;
bool wifiConnected = false;
bool websocketConnected = false;

unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 seconds

unsigned long lastIMURead = 0;
const unsigned long IMU_READ_INTERVAL = 100; // 100ms = 10 Hz

// ============================================================================
// WEBSOCKET EVENT HANDLER
// ============================================================================

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WebSocket] Disconnected");
      websocketConnected = false;
      digitalWrite(STATUS_LED, LOW);
      break;

    case WStype_CONNECTED:
      Serial.println("[WebSocket] Connected to server");
      websocketConnected = true;
      digitalWrite(STATUS_LED, HIGH);

      // Send initial connection message
      sendAcknowledgment("000", "ok", "ESP32 WiFi bridge connected");
      break;

    case WStype_TEXT:
      {
        Serial.printf("[WebSocket] Received: %s\n", payload);

        // Parse JSON command
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, payload, length);

        if (error) {
          Serial.print("[WebSocket] JSON parse error: ");
          Serial.println(error.c_str());
          return;
        }

        // Forward to Arduino via Serial
        forwardCommandToArduino(doc);
      }
      break;

    case WStype_ERROR:
      Serial.println("[WebSocket] Error occurred");
      break;

    default:
      break;
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n===========================================");
  Serial.println("ELEGOO WiFi Bridge - Starting...");
  Serial.println("===========================================\n");

  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Initialize Serial communication with Arduino Uno
  ARDUINO_SERIAL.begin(SERIAL_BAUD, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("[Arduino] Serial initialized (9600 baud)");

  // Connect to WiFi
  connectToWiFi();

  // Initialize WebSocket connection
  if (wifiConnected) {
    webSocket.begin(SERVER_HOST, SERVER_PORT, WEBSOCKET_PATH);
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000); // Reconnect every 5 seconds if disconnected
    Serial.println("[WebSocket] Client initialized");
  }

  Serial.println("\n===========================================");
  Serial.println("ELEGOO WiFi Bridge - Ready!");
  Serial.println("===========================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Handle WebSocket events
  if (wifiConnected) {
    webSocket.loop();
  }

  // Read data from Arduino (acknowledgments, IMU data)
  readFromArduino();

  // Send periodic heartbeat
  unsigned long currentMillis = millis();
  if (websocketConnected && (currentMillis - lastHeartbeat > HEARTBEAT_INTERVAL)) {
    lastHeartbeat = currentMillis;
    // Heartbeat is implicit through IMU data or you can send explicit ping
  }

  // Request IMU data from Arduino periodically
  if (currentMillis - lastIMURead > IMU_READ_INTERVAL) {
    lastIMURead = currentMillis;
    // The ELEGOO Arduino continuously outputs data, so we just read it
  }

  // Small delay to prevent overwhelming the CPU
  delay(10);
}

// ============================================================================
// WIFI CONNECTION
// ============================================================================

void connectToWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.print(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); // Blink LED
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP address: ");
    Serial.println(WiFi.localIP());
    digitalWrite(STATUS_LED, HIGH);
  } else {
    wifiConnected = false;
    Serial.println("\n[WiFi] Failed to connect!");
    digitalWrite(STATUS_LED, LOW);
  }
}

// ============================================================================
// COMMAND FORWARDING TO ARDUINO
// ============================================================================

void forwardCommandToArduino(JsonDocument& doc) {
  // Extract command ID for acknowledgment
  const char* commandId = doc["H"] | "000";

  // Serialize JSON and send to Arduino via Serial
  String jsonString;
  serializeJson(doc, jsonString);

  ARDUINO_SERIAL.println(jsonString);

  Serial.print("[Arduino] Sent command: ");
  Serial.println(jsonString);

  // Send acknowledgment to server (optimistic - assuming Arduino will execute)
  // Real acknowledgment should come from Arduino, but for simplicity we send immediately
  sendAcknowledgment(commandId, "ok", "Command forwarded to Arduino");
}

// ============================================================================
// READ FROM ARDUINO
// ============================================================================

void readFromArduino() {
  // Read any data from Arduino (could be acknowledgments or IMU data)
  while (ARDUINO_SERIAL.available()) {
    String line = ARDUINO_SERIAL.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      Serial.print("[Arduino] Received: ");
      Serial.println(line);

      // Try to parse as JSON
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, line);

      if (!error) {
        // Check if it's IMU data
        if (doc.containsKey("yaw") || doc.containsKey("pitch") || doc.containsKey("roll")) {
          sendIMUData(doc);
        }
        // Check if it's an acknowledgment
        else if (doc.containsKey("ack")) {
          const char* commandId = doc["ack"];
          sendAcknowledgment(commandId, "ok", "Arduino confirmed");
        }
      }
      // If not JSON, might be debugging output - forward to serial
    }
  }
}

// ============================================================================
// SEND IMU DATA TO SERVER
// ============================================================================

void sendIMUData(JsonDocument& imuDoc) {
  if (!websocketConnected) return;

  // Create IMU message
  StaticJsonDocument<256> doc;
  doc["type"] = "imu";
  doc["yaw"] = imuDoc["yaw"] | 0.0;
  doc["pitch"] = imuDoc["pitch"] | 0.0;
  doc["roll"] = imuDoc["roll"] | 0.0;

  // Serialize and send
  String jsonString;
  serializeJson(doc, jsonString);
  webSocket.sendTXT(jsonString);

  Serial.print("[WebSocket] Sent IMU: ");
  Serial.println(jsonString);
}

// ============================================================================
// SEND ACKNOWLEDGMENT TO SERVER
// ============================================================================

void sendAcknowledgment(const char* commandId, const char* status, const char* message = "") {
  if (!websocketConnected) return;

  StaticJsonDocument<256> doc;
  doc["command_id"] = commandId;
  doc["status"] = status;
  if (strlen(message) > 0) {
    doc["message"] = message;
  }

  String jsonString;
  serializeJson(doc, jsonString);
  webSocket.sendTXT(jsonString);

  Serial.print("[WebSocket] Sent ACK: ");
  Serial.println(jsonString);
}
