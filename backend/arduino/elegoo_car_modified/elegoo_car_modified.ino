/**
 * ELEGOO Smart Robot Car V4.0 - Modified for WiFi Control
 *
 * This is a modified version of the ELEGOO stock firmware that:
 * 1. Accepts JSON motor commands from ESP32 WiFi bridge via Serial
 * 2. Outputs IMU (MPU6050) data as JSON
 * 3. Sends acknowledgments for commands
 *
 * Hardware:
 * - ELEGOO Smart Robot Car V4.0 kit
 * - Arduino Uno
 * - L298N Motor Driver
 * - MPU6050 IMU (gyro/accelerometer)
 * - Ultrasonic sensor (optional)
 * - Line tracking sensors (optional)
 *
 * Serial Communication with ESP32:
 * - Receives JSON commands: {"N": 4, "H": "cmd123", "D1": 200, "D2": 200}
 * - Sends IMU data: {"yaw": 45.2, "pitch": 0.1, "roll": -1.3}
 * - Sends acknowledgments: {"ack": "cmd123"}
 */

#include <Wire.h>
#include <ArduinoJson.h>

// ============================================================================
// MPU6050 IMU CONFIGURATION
// ============================================================================

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 registers
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_ACCEL_XOUT_H 0x3B

// IMU calibration offsets (you may need to calibrate these)
float gyroXoffset = 0;
float gyroYoffset = 0;
float gyroZoffset = 0;

// Current orientation (degrees)
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

unsigned long lastIMUUpdate = 0;
const unsigned long IMU_UPDATE_INTERVAL = 100; // 10 Hz

// ============================================================================
// MOTOR CONTROL CONFIGURATION
// ============================================================================

// L298N Motor Driver Pins (adjust based on your wiring)
#define ENA 5   // Left motor PWM
#define ENB 6   // Right motor PWM
#define IN1 7   // Left motor direction 1
#define IN2 8   // Left motor direction 2
#define IN3 9   // Right motor direction 1
#define IN4 11  // Right motor direction 2

// Motor speed limits
#define MIN_SPEED 0
#define MAX_SPEED 255

// Current motor speeds
int leftSpeed = 0;
int rightSpeed = 0;

// ============================================================================
// COMMAND PROTOCOL
// ============================================================================

/**
 * Command Format (JSON):
 * {
 *   "N": <command_type>,
 *   "H": "<command_id>",
 *   "D1": <left_motor_speed>,
 *   "D2": <right_motor_speed>
 * }
 *
 * Command Types:
 * N=1: Move forward
 * N=2: Move backward
 * N=3: Turn left
 * N=4: Turn right
 * N=5: Stop
 * N=6: Set motor speeds (D1=left, D2=right, signed -255 to 255)
 */

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize Serial communication (for ESP32 WiFi bridge)
  Serial.begin(9600);

  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Stop motors initially
  stopMotors();

  // Initialize MPU6050 IMU
  Wire.begin();
  initMPU6050();

  // Calibrate gyro (car must be stationary!)
  Serial.println("{\"status\":\"Calibrating gyro...\"}");
  calibrateGyro();

  Serial.println("{\"status\":\"ELEGOO Car Ready\"}");
  delay(1000);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Read and process commands from Serial (ESP32)
  if (Serial.available() > 0) {
    String jsonString = Serial.readStringUntil('\n');
    jsonString.trim();

    if (jsonString.length() > 0) {
      processCommand(jsonString);
    }
  }

  // Update IMU and send data periodically
  unsigned long currentMillis = millis();
  if (currentMillis - lastIMUUpdate >= IMU_UPDATE_INTERVAL) {
    lastIMUUpdate = currentMillis;
    updateIMU();
    sendIMUData();
  }

  delay(10); // Small delay to prevent overwhelming CPU
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void processCommand(String jsonString) {
  // Parse JSON command
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    Serial.print("{\"error\":\"JSON parse failed: ");
    Serial.print(error.c_str());
    Serial.println("\"}");
    return;
  }

  // Extract command parameters
  int commandType = doc["N"] | 0;
  const char* commandId = doc["H"] | "000";
  int data1 = doc["D1"] | 0;
  int data2 = doc["D2"] | 0;

  // Execute command
  switch (commandType) {
    case 1: // Forward
      moveForward(data1 > 0 ? data1 : 200);
      break;

    case 2: // Backward
      moveBackward(data1 > 0 ? data1 : 200);
      break;

    case 3: // Turn left
      turnLeft(data1 > 0 ? data1 : 200);
      break;

    case 4: // Turn right
      turnRight(data1 > 0 ? data1 : 200);
      break;

    case 5: // Stop
      stopMotors();
      break;

    case 6: // Set motor speeds directly (D1=left, D2=right)
      setMotorSpeeds(data1, data2);
      break;

    default:
      Serial.println("{\"error\":\"Unknown command type\"}");
      return;
  }

  // Send acknowledgment
  sendAcknowledgment(commandId);
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

void setMotorSpeeds(int leftMotorSpeed, int rightMotorSpeed) {
  // Constrain speeds to valid range
  leftMotorSpeed = constrain(leftMotorSpeed, -MAX_SPEED, MAX_SPEED);
  rightMotorSpeed = constrain(rightMotorSpeed, -MAX_SPEED, MAX_SPEED);

  // Left motor
  if (leftMotorSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftMotorSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftMotorSpeed);
  }

  // Right motor
  if (rightMotorSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightMotorSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightMotorSpeed);
  }

  leftSpeed = leftMotorSpeed;
  rightSpeed = rightMotorSpeed;
}

void moveForward(int speed) {
  setMotorSpeeds(speed, speed);
}

void moveBackward(int speed) {
  setMotorSpeeds(-speed, -speed);
}

void turnLeft(int speed) {
  setMotorSpeeds(-speed, speed); // Left backward, right forward
}

void turnRight(int speed) {
  setMotorSpeeds(speed, -speed); // Left forward, right backward
}

void stopMotors() {
  setMotorSpeeds(0, 0);
}

// ============================================================================
// MPU6050 IMU FUNCTIONS
// ============================================================================

void initMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0); // Set to 0 to wake up
  Wire.endTransmission(true);

  delay(100);
}

void calibrateGyro() {
  const int numSamples = 100;
  long sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);

    int16_t gyroX = Wire.read() << 8 | Wire.read();
    int16_t gyroY = Wire.read() << 8 | Wire.read();
    int16_t gyroZ = Wire.read() << 8 | Wire.read();

    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;

    delay(10);
  }

  gyroXoffset = sumX / (float)numSamples;
  gyroYoffset = sumY / (float)numSamples;
  gyroZoffset = sumZ / (float)numSamples;

  Serial.println("{\"status\":\"Gyro calibrated\"}");
}

void updateIMU() {
  // Read accelerometer data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  int16_t accelX = Wire.read() << 8 | Wire.read();
  int16_t accelY = Wire.read() << 8 | Wire.read();
  int16_t accelZ = Wire.read() << 8 | Wire.read();

  // Read gyroscope data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();

  // Apply calibration offsets
  float gyroXrate = (gyroX - gyroXoffset) / 131.0; // 131 LSB/°/s for ±250°/s range
  float gyroYrate = (gyroY - gyroYoffset) / 131.0;
  float gyroZrate = (gyroZ - gyroZoffset) / 131.0;

  // Calculate pitch and roll from accelerometer
  float accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float accelRoll = atan2(-accelX, accelZ) * 180.0 / PI;

  // Integrate gyro for yaw (heading)
  float dt = IMU_UPDATE_INTERVAL / 1000.0; // seconds
  yaw += gyroZrate * dt;

  // Keep yaw in 0-360 range
  if (yaw >= 360.0) yaw -= 360.0;
  if (yaw < 0.0) yaw += 360.0;

  // Complementary filter for pitch and roll (95% gyro, 5% accel)
  pitch = 0.95 * (pitch + gyroXrate * dt) + 0.05 * accelPitch;
  roll = 0.95 * (roll + gyroYrate * dt) + 0.05 * accelRoll;
}

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================

void sendIMUData() {
  StaticJsonDocument<128> doc;
  doc["yaw"] = yaw;
  doc["pitch"] = pitch;
  doc["roll"] = roll;

  serializeJson(doc, Serial);
  Serial.println();
}

void sendAcknowledgment(const char* commandId) {
  StaticJsonDocument<64> doc;
  doc["ack"] = commandId;

  serializeJson(doc, Serial);
  Serial.println();
}
