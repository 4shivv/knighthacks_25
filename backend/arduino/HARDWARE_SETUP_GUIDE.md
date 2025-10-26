# Hardware Setup Guide - ELEGOO Smart Car WiFi Control

Complete guide to setting up the autonomous navigation system with ELEGOO Smart Robot Car V4.0.

---

## 📦 Required Hardware

### Main Components:
1. **ELEGOO Smart Robot Car V4.0** (complete kit)
   - Arduino Uno
   - L298N Motor Driver
   - MPU6050 IMU (gyro/accelerometer)
   - 4x DC motors with wheels
   - Ultrasonic sensor (optional)
   - Line tracking sensors (optional)

2. **ESP32 Development Board**
   - Any ESP32 board with WiFi (e.g., ESP32-DevKitC, NodeMCU-32S)
   - USB cable for programming

3. **iPhone with LiDAR** (iPhone 12 Pro or newer)
   - For 3D environment scanning

4. **ESP32-CAM** (or similar)
   - 2MP camera for object detection
   - USB-to-Serial adapter for programming

5. **Backend Server**
   - Laptop/Desktop running Python 3.9+
   - Same WiFi network as ESP32

---

## 🔌 Wiring Diagram

### ESP32 WiFi Bridge → Arduino Uno

**Serial Communication:**
```
ESP32          Arduino Uno
─────────────  ──────────────
TX2 (GPIO17) → RX (Pin 0)
RX2 (GPIO16) → TX (Pin 1)
GND          → GND
5V           → 5V (or VIN)
```

**Important Notes:**
- Use ESP32's **Serial2** (not Serial) to avoid conflicts with USB
- Arduino Pin 0/1 are the hardware serial pins
- Make sure GND is connected between both boards
- Power can come from shared battery or separate sources

### Motor Driver (L298N) → Arduino Uno

**Standard ELEGOO V4.0 Wiring:**
```
L298N          Arduino Uno
─────────────  ──────────────
ENA           → Pin 5 (PWM)
IN1           → Pin 7
IN2           → Pin 8
IN3           → Pin 9
IN4           → Pin 11
ENB           → Pin 6 (PWM)
```

**Power:**
```
L298N          Battery
─────────────  ──────────────
12V           → 7.4V-12V (battery +)
GND           → Battery (-)
5V OUT        → Arduino 5V (optional)
```

### MPU6050 IMU → Arduino Uno

**I2C Connection:**
```
MPU6050        Arduino Uno
─────────────  ──────────────
VCC           → 5V
GND           → GND
SCL           → A5 (SCL)
SDA           → A4 (SDA)
```

### ESP32-CAM → WiFi

**No wiring needed** - communicates via WiFi to backend server.

**Programming Setup:**
```
ESP32-CAM      FTDI USB-Serial
─────────────  ──────────────
5V            → 5V
GND           → GND
U0T (TX)      → RX
U0R (RX)      → TX
IO0           → GND (during upload)
```

---

## 💻 Software Installation

### Step 1: Arduino IDE Setup

1. **Install Arduino IDE** (version 1.8.19 or 2.x)
   - Download from: https://www.arduino.cc/en/software

2. **Install ESP32 Board Support:**
   - Open Arduino IDE
   - Go to: `File > Preferences`
   - Add to "Additional Boards Manager URLs":
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - Go to: `Tools > Board > Boards Manager`
   - Search "esp32" and install "esp32 by Espressif Systems"

3. **Install Required Libraries:**
   - Go to: `Sketch > Include Library > Manage Libraries`
   - Install the following:
     - `ArduinoJson` (by Benoit Blanchon) - version 6.x
     - `WebSockets` (by Markus Sattler)
     - `Wire` (built-in for I2C)

### Step 2: Upload ESP32 WiFi Bridge Code

1. **Open the sketch:**
   ```
   arduino/elegoo_wifi_bridge/elegoo_wifi_bridge.ino
   ```

2. **Configure WiFi and Server:**
   Edit these lines (around line 28):
   ```cpp
   const char* WIFI_SSID = "YOUR_WIFI_SSID";        // Your WiFi name
   const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"; // Your WiFi password
   const char* SERVER_HOST = "192.168.1.100";       // Your laptop IP
   const int SERVER_PORT = 8000;
   ```

3. **Select Board:**
   - `Tools > Board > ESP32 Arduino > ESP32 Dev Module`
   - `Tools > Port > [Your ESP32 port]`

4. **Upload:**
   - Click "Upload" button
   - Wait for "Hard resetting via RTS pin..."

5. **Verify:**
   - Open Serial Monitor (115200 baud)
   - Should see:
     ```
     [WiFi] Connected!
     [WiFi] IP address: 192.168.1.XXX
     [WebSocket] Connected to server
     ```

### Step 3: Upload Modified ELEGOO Car Code

1. **IMPORTANT: Disconnect ESP32 TX/RX from Arduino first!**
   - Arduino Pin 0/1 are used for programming
   - If ESP32 is connected, upload will fail

2. **Open the sketch:**
   ```
   arduino/elegoo_car_modified/elegoo_car_modified.ino
   ```

3. **Verify Motor Pins:**
   Edit lines 40-45 if your wiring is different:
   ```cpp
   #define ENA 5   // Left motor PWM
   #define ENB 6   // Right motor PWM
   #define IN1 7   // Left motor direction 1
   #define IN2 8   // Left motor direction 2
   #define IN3 9   // Right motor direction 1
   #define IN4 11  // Right motor direction 2
   ```

4. **Select Board:**
   - `Tools > Board > Arduino AVR Boards > Arduino Uno`
   - `Tools > Port > [Your Arduino port]`

5. **Upload:**
   - Click "Upload" button
   - Wait for "Done uploading"

6. **Reconnect ESP32 TX/RX to Arduino Pin 0/1**

7. **Calibrate IMU:**
   - Place car on flat surface (don't move it!)
   - Power on Arduino
   - IMU will auto-calibrate for 1-2 seconds
   - LED on Arduino should blink, then stay on

---

## 🚀 Backend Server Setup

### Step 1: Install Python Dependencies

```bash
cd backend
pip install -r requirements.txt
```

**Key dependencies:**
- `fastapi` - Web server
- `uvicorn` - ASGI server
- `opencv-python` - Image processing
- `ultralytics` - YOLO11
- `google-genai` - Gemini ADK
- `numpy` - Math operations

### Step 2: Download YOLO Model

The YOLO model will auto-download on first run, but you can pre-download:

```bash
cd backend
python -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"
```

### Step 3: Configure Backend

Edit `backend/config.py` if needed:

```python
# Server settings
server_host = "0.0.0.0"  # Listen on all interfaces
server_port = 8000

# YOLO settings
yolo_model_path = "yolo11n.pt"
confidence_threshold = 0.7  # Adjust detection sensitivity

# Gemini API key (set in environment or here)
gemini_api_key = os.getenv("GOOGLE_API_KEY")
```

### Step 4: Set Gemini API Key

**Option A: Environment Variable (Recommended)**
```bash
export GOOGLE_API_KEY="your-gemini-api-key"
```

**Option B: Create `.env` file**
```bash
cd backend
echo "GOOGLE_API_KEY=your-gemini-api-key" > .env
```

Get your key from: https://aistudio.google.com/apikey

### Step 5: Start Backend Server

```bash
cd backend
python main.py
```

**Expected output:**
```
============================================================
SERVER INITIALIZATION STARTED
============================================================

Step 1: Loading YOLO model...
✓ YOLO model loaded successfully

Step 2: Initializing ADK Navigation Agent...
✓ ADK Navigation Agent initialized successfully

Step 3: Initializing shared state manager...
✓ Mission started: Find the target object

Step 4: Starting ADK session...
✓ ADK session started successfully

Step 5: Starting concurrent loops...
✓ Detection loop started (3000ms interval)
✓ Planning loop started (1000ms interval)
✓ Motor control loop started (3000ms interval)
✓ Path planning loop started (1000ms interval)
✓ Sensor fusion loop started (500ms interval)

============================================================
SERVER READY - Waiting for sensor input...
============================================================

INFO:     Uvicorn running on http://0.0.0.0:8000
```

---

## 📱 Frontend Setup

### Step 1: Open Dashboard

1. Open `frontenddashboard.html` in Chrome/Firefox
2. Click "Configure Backend"
3. Enter backend URL: `http://YOUR_LAPTOP_IP:8000`
4. Click "Test & Save"
5. Should see "Connected" status

### Step 2: Connect iPhone LiDAR

1. Open iOS LiDAR capture app on iPhone
2. Configure backend URL: `http://YOUR_LAPTOP_IP:8000`
3. Start scanning
4. Dashboard should show "LiDAR Scanning..."

### Step 3: Connect ESP32-CAM

1. Flash ESP32-CAM with camera streaming code
2. Configure WiFi and backend URL
3. Power on ESP32-CAM
4. Dashboard should show live video stream

---

## ✅ Testing Checklist

### Hardware Tests:

- [ ] **ESP32 WiFi Bridge:**
  - [ ] Connects to WiFi
  - [ ] Connects to backend WebSocket
  - [ ] LED on ESP32 is solid (not blinking)

- [ ] **Arduino Car:**
  - [ ] Powers on
  - [ ] IMU calibrates (car must be still)
  - [ ] Motors respond to test commands

- [ ] **Motor Test:**
  - [ ] Forward/backward movement
  - [ ] Left/right turning
  - [ ] Stop command works
  - [ ] No unusual sounds/heat

- [ ] **IMU Test:**
  - [ ] Sends data to backend
  - [ ] Yaw updates when car rotates
  - [ ] Pitch/roll update when tilted

### Software Tests:

- [ ] **Backend:**
  - [ ] Server starts without errors
  - [ ] All 5 loops running
  - [ ] YOLO model loaded
  - [ ] Gemini ADK initialized

- [ ] **Frontend:**
  - [ ] Dashboard loads in browser
  - [ ] Connects to backend
  - [ ] Video stream shows
  - [ ] Mission control buttons work

- [ ] **End-to-End:**
  - [ ] Start mission from dashboard
  - [ ] YOLO detects objects (bounding boxes appear)
  - [ ] LiDAR data creates occupancy grid
  - [ ] Path planning finds route
  - [ ] Motor commands sent to car
  - [ ] Car moves autonomously!

---

## 🐛 Troubleshooting

### ESP32 won't connect to WiFi:
- Check SSID/password are correct
- Make sure WiFi is 2.4GHz (ESP32 doesn't support 5GHz)
- Try moving closer to router
- Check Serial Monitor for error messages

### Arduino upload fails:
- Disconnect ESP32 TX/RX from Arduino Pin 0/1
- Select correct board: "Arduino Uno"
- Select correct COM port
- Try different USB cable

### Motors don't move:
- Check battery voltage (should be 7-12V)
- Verify L298N wiring
- Test motors directly with battery
- Check if motor driver is getting hot (may be shorted)

### IMU data is noisy:
- Recalibrate on flat surface
- Check I2C connections (SDA/SCL)
- Reduce vibration (add padding under MPU6050)

### WebSocket disconnects:
- Check firewall settings
- Make sure backend is running
- Verify IP address is correct
- Try restarting ESP32

### YOLO not detecting objects:
- Increase lighting
- Adjust confidence threshold in config.py
- Check camera focus
- Verify camera stream is working

### Path planning doesn't work:
- Verify LiDAR data is being received
- Check occupancy grid is populated
- Ensure sensor fusion loop is running
- Review backend logs for errors

---

## 📊 System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    iPhone (LiDAR)                       │
│                         │                               │
│                    WiFi (WebSocket)                     │
│                         ↓                               │
│  ┌──────────────────────────────────────────────────┐  │
│  │         Backend Server (Python/FastAPI)          │  │
│  │                                                  │  │
│  │  ┌─────────────────────────────────────────┐   │  │
│  │  │  Detection Loop (YOLO11n)               │   │  │
│  │  │  Planning Loop (Gemini ADK)             │   │  │
│  │  │  Fusion Loop (LiDAR + Camera → Grid)    │   │  │
│  │  │  Path Planning Loop (D* Lite)           │   │  │
│  │  │  Motor Control Loop                     │   │  │
│  │  └─────────────────────────────────────────┘   │  │
│  │                       ↓                         │  │
│  │                 WebSocket (/ws/arduino)         │  │
│  └──────────────────────────────────────────────────┘  │
│                         │                               │
│                    WiFi Network                         │
│                         ↓                               │
│  ┌──────────────────────────────────────────────────┐  │
│  │         ESP32 WiFi Bridge                        │  │
│  │  (Forwards commands to Arduino via Serial)      │  │
│  └──────────────────────────────────────────────────┘  │
│                         │                               │
│                   Serial (TX/RX)                        │
│                         ↓                               │
│  ┌──────────────────────────────────────────────────┐  │
│  │      Arduino Uno (ELEGOO Car)                    │  │
│  │                                                  │  │
│  │  - Receives JSON motor commands                 │  │
│  │  - Controls L298N motor driver                  │  │
│  │  - Reads MPU6050 IMU                            │  │
│  │  - Sends IMU data back to ESP32                 │  │
│  └──────────────────────────────────────────────────┘  │
│                         │                               │
│                         ↓                               │
│            4x DC Motors + Wheels (Movement)             │
└─────────────────────────────────────────────────────────┘
```

---

## 🎯 Next Steps After Setup

1. **Calibrate the system:**
   - Test motor speeds (adjust if car veers left/right)
   - Calibrate LiDAR-camera alignment
   - Tune YOLO confidence threshold

2. **Test missions:**
   - Start with simple: "Move forward 1 meter"
   - Then: "Find a cup"
   - Advanced: "Navigate to the red mug, avoid obstacles"

3. **Monitor performance:**
   - Check Activity Log for errors
   - View backend logs for debugging
   - Measure mission success rate

4. **Optimize:**
   - Adjust loop intervals for faster response
   - Tune D* Lite heuristics
   - Improve sensor fusion accuracy

---

## 📚 Additional Resources

- **ELEGOO Manual**: [Official V4.0 Kit Documentation]
- **ESP32 Docs**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **YOLO11 Docs**: https://docs.ultralytics.com/
- **Gemini ADK**: https://ai.google.dev/gemini-api/docs

---

## ⚠️ Safety Notes

1. **Test in safe area** - Use a large open space
2. **Emergency stop ready** - Keep "Stop Mission" button accessible
3. **Battery safety** - Don't overcharge LiPo batteries
4. **Motor heat** - Let motors cool if they get hot
5. **WiFi range** - Stay within 20-30 feet of router

---

**You're all set!** 🚀 Your autonomous navigation rover is ready to explore!
