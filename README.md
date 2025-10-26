# 🤖 Blindfolded Navigation - AI Autonomous Robot Car

An AI-powered autonomous navigation system that enables an ELEGOO Smart Robot Car to find objects in unknown environments using computer vision, LiDAR mapping, and intelligent path planning.

![Status](https://img.shields.io/badge/status-production%20ready-brightgreen)
![Python](https://img.shields.io/badge/python-3.9%2B-blue)
![FastAPI](https://img.shields.io/badge/FastAPI-0.100%2B-009688)
![License](https://img.shields.io/badge/license-MIT-blue)

---

## 🎯 What It Does

Give your robot a mission like **"Find the red coffee mug"** and watch it:

1. 🎥 **See** - Uses YOLO11n AI to detect objects in real-time
2. 🗺️ **Map** - Combines iPhone LiDAR + ESP32 camera for 3D environment mapping
3. 🧠 **Think** - Google Gemini 2.0 Flash AI makes navigation decisions
4. 🚗 **Navigate** - D\* Lite algorithm plans optimal paths around obstacles
5. ✅ **Succeed** - Autonomously drives to the target object

**No pre-mapping required!** The robot learns its environment on the fly.

---

## 📹 Demo

```
User: "Find the red mug"
  ↓
[Robot scans environment with camera + LiDAR]
  ↓
[YOLO detects objects, Gemini decides which to investigate]
  ↓
[D* Lite plans path around obstacles]
  ↓
[Robot navigates autonomously]
  ↓
Robot: "Target found! ✓"
```

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      iPhone (LiDAR)                         │
│                 Scans 360° environment                      │
└──────────────────────────┬──────────────────────────────────┘
                           │ WiFi WebSocket
                           ↓
┌─────────────────────────────────────────────────────────────┐
│              Backend Server (Python/FastAPI)                │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  🔄 Detection Loop (YOLO11n)     - 3-5s            │   │
│  │  🔄 Planning Loop (Gemini ADK)   - 1s              │   │
│  │  🔄 Fusion Loop (Sensor Fusion)  - 500ms           │   │
│  │  🔄 Path Planning (D* Lite)      - 1s              │   │
│  │  🔄 Motor Control Loop           - 3s              │   │
│  └─────────────────────────────────────────────────────┘   │
└──────────────────────────┬──────────────────────────────────┘
                           │ WiFi WebSocket
                           ↓
┌─────────────────────────────────────────────────────────────┐
│                    ESP32 WiFi Bridge                        │
│              Forwards commands via Serial                   │
└──────────────────────────┬──────────────────────────────────┘
                           │ Serial TX/RX
                           ↓
┌─────────────────────────────────────────────────────────────┐
│           Arduino Uno (ELEGOO Smart Car)                    │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  • Receives JSON motor commands                     │   │
│  │  • Controls L298N motor driver                      │   │
│  │  • Reads MPU6050 IMU (heading)                      │   │
│  │  • Sends telemetry back to backend                  │   │
│  └─────────────────────────────────────────────────────┘   │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ↓
                    4x DC Motors
                    CAR MOVES! 🚗💨
```

---

## ⚡ Quick Start

### Prerequisites

**Hardware:**
- ELEGOO Smart Robot Car V4.0 kit
- ESP32 development board
- iPhone with LiDAR (iPhone 12 Pro or newer)
- ESP32-CAM (optional, for camera)
- Laptop/Desktop (backend server)

**Software:**
- Python 3.9 or higher
- Arduino IDE 1.8.19 or 2.x
- Google Gemini API key ([Get one here](https://aistudio.google.com/apikey))
- Web browser (Chrome/Firefox recommended)

### Installation (5 minutes)

```bash
# 1. Clone repository
git clone https://github.com/yourusername/blindfolded-navigation.git
cd blindfolded-navigation

# 2. Install Python dependencies
cd backend
pip install -r requirements.txt

# 3. Set Gemini API key
export GOOGLE_API_KEY="your-gemini-api-key-here"

# 4. Start backend server
python main.py
```

**That's it!** The backend is now running on `http://0.0.0.0:8000`

---

## 📚 Table of Contents

1. [Backend Setup](#-backend-setup)
2. [Frontend Setup](#-frontend-setup)
3. [Arduino Setup](#-arduino-setup)
4. [System Workflow](#-system-workflow)
5. [API Documentation](#-api-documentation)
6. [Troubleshooting](#-troubleshooting)
7. [Architecture Details](#-architecture-details)

---

## 🖥️ Backend Setup

### Step 1: Install Dependencies

```bash
cd backend

# Install Python packages
pip install -r requirements.txt
```

**Required packages:**
- `fastapi` - Web framework
- `uvicorn` - ASGI server
- `opencv-python` - Image processing
- `ultralytics` - YOLO11
- `google-genai` - Gemini ADK
- `numpy` - Math operations
- `websockets` - WebSocket support

### Step 2: Configure Environment

Create a `.env` file:

```bash
# backend/.env
GOOGLE_API_KEY=your-gemini-api-key-here
```

Or export as environment variable:

```bash
export GOOGLE_API_KEY="your-gemini-api-key-here"
```

### Step 3: Configure Settings

Edit `backend/config.py` if needed:

```python
class Settings:
    # Server
    server_host = "0.0.0.0"  # Listen on all interfaces
    server_port = 8000

    # YOLO
    yolo_model_path = "yolo11n.pt"  # Auto-downloads
    confidence_threshold = 0.7       # Detection threshold

    # Mission
    default_objective = "Find the target object"

    # Loop intervals (milliseconds)
    detection_loop_interval = 3000   # 3 seconds
    planning_loop_interval = 1000    # 1 second
    fusion_loop_interval = 500       # 500ms
```

### Step 4: Start Backend

```bash
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

### Backend File Structure

```
backend/
├── main.py                     # FastAPI server entry point
├── config.py                   # Configuration settings
├── state_manager.py            # Thread-safe state management
├── adk_session_manager.py      # Gemini ADK session handler
│
├── loops/                      # Concurrent processing loops
│   ├── detection_loop.py       # YOLO object detection
│   ├── planning_loop.py        # Gemini AI planning
│   ├── fusion_loop.py          # Sensor fusion (LiDAR+Camera)
│   ├── path_planning_loop.py   # D* Lite pathfinding
│   └── motor_control_loop.py   # Motor command generation
│
├── models/
│   └── yolo_model.py           # YOLO11n wrapper
│
├── agents/
│   └── navigation_agent.py     # Gemini agent definition
│
├── pathfinding/
│   ├── d_star_lite.py          # D* Lite algorithm
│   ├── occupancy_grid.py       # Grid representation
│   ├── priority_queue.py       # Min-heap for pathfinding
│   └── utils.py                # Coordinate utilities
│
└── tools/
    ├── odometry.py             # Position tracking
    ├── motor_command_translator.py
    └── navigation_tools.py
```

---

## 🌐 Frontend Setup

### Step 1: Open Dashboard

**No installation required!** Just open the HTML file:

```bash
# From project root
open frontenddashboard.html

# Or drag-and-drop into Chrome/Firefox
```

### Step 2: Configure Backend URL

1. Click **"Configure Backend"** (gear icon)
2. Enter your laptop's IP address:
   ```
   http://192.168.1.100:8000
   ```
   (Replace `192.168.1.100` with your actual IP)

3. Click **"Test & Save"**
4. Status should show: ✅ **Connected**

### Step 3: Start Mission

1. Enter objective: `"Find the red mug"`
2. Click **"Start Mission"**
3. Watch the robot navigate autonomously!

### Frontend Features

- **📹 Live Camera Feed** - Real-time video from ESP32-CAM
- **🎯 Bounding Boxes** - YOLO detections with labels and depth
- **📊 Activity Log** - Real-time backend updates
- **🎮 Mission Control** - Start/stop autonomous navigation
- **📡 LiDAR Status** - Scan counter and depth data
- **📈 Progress Bar** - Mission completion percentage

### Frontend File Structure

```
frontenddashboard.html          # Single-page React app
├── Mission Control UI
├── Live Video Stream with Canvas Overlay
├── Activity Log (auto-updates every 500ms)
└── System Status Dashboard
```

---

## 🔧 Arduino Setup

### Hardware Wiring

#### ESP32 WiFi Bridge → Arduino Uno

```
ESP32          Arduino Uno
─────────────  ──────────────
TX2 (GPIO17) → RX (Pin 0)
RX2 (GPIO16) → TX (Pin 1)
GND          → GND
5V           → 5V
```

#### L298N Motor Driver → Arduino Uno

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

### Software Setup

#### Part 1: Upload ESP32 WiFi Bridge

1. **Install ESP32 Board Support:**
   - Arduino IDE → `File > Preferences`
   - Add to "Additional Boards Manager URLs":
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - `Tools > Board > Boards Manager` → Install "esp32"

2. **Install Libraries:**
   - `Sketch > Include Library > Manage Libraries`
   - Install: `ArduinoJson` (6.x), `WebSockets`

3. **Configure WiFi:**
   - Open `backend/arduino/elegoo_wifi_bridge/elegoo_wifi_bridge.ino`
   - Edit lines 28-32:
     ```cpp
     const char* WIFI_SSID = "YourWiFiName";
     const char* WIFI_PASSWORD = "YourPassword";
     const char* SERVER_HOST = "192.168.1.100";  // Your laptop IP
     ```

4. **Upload:**
   - `Tools > Board > ESP32 Dev Module`
   - `Tools > Port > [Your ESP32 port]`
   - Click **Upload**

5. **Verify:**
   - Open Serial Monitor (115200 baud)
   - Should see: `[WiFi] Connected!` and `[WebSocket] Connected to server`

#### Part 2: Upload ELEGOO Car Code

1. **IMPORTANT:** Disconnect ESP32 TX/RX from Arduino Pin 0/1 first!

2. **Open sketch:**
   - `backend/arduino/elegoo_car_modified/elegoo_car_modified.ino`

3. **Verify motor pins** (lines 40-45):
   ```cpp
   #define ENA 5   // Left motor PWM
   #define ENB 6   // Right motor PWM
   #define IN1 7   // Left motor direction 1
   #define IN2 8   // Left motor direction 2
   #define IN3 9   // Right motor direction 1
   #define IN4 11  // Right motor direction 2
   ```

4. **Upload:**
   - `Tools > Board > Arduino Uno`
   - `Tools > Port > [Your Arduino port]`
   - Click **Upload**

5. **Reconnect ESP32 TX/RX to Arduino Pin 0/1**

6. **Calibrate IMU:**
   - Place car on flat surface (don't move!)
   - Power on
   - Wait 2 seconds for auto-calibration

### Arduino File Structure

```
backend/arduino/
├── elegoo_wifi_bridge/
│   └── elegoo_wifi_bridge.ino      # ESP32 WiFi bridge (upload to ESP32)
│
├── elegoo_car_modified/
│   └── elegoo_car_modified.ino     # Modified ELEGOO code (upload to Arduino Uno)
│
└── HARDWARE_SETUP_GUIDE.md         # Detailed wiring diagrams
```

---

## 🔄 System Workflow

### Complete Data Flow (From User Click to Car Movement)

#### 1️⃣ **User Starts Mission**

```
User opens frontenddashboard.html
    ↓
User types: "Find the red mug"
    ↓
User clicks: "Start Mission"
    ↓
Frontend sends: POST /api/start_mission
{
  "mission": "Find the red mug",
  "timestamp": "2025-10-26T12:00:00Z"
}
    ↓
Backend receives and activates:
✓ state_manager.mission_active = True
✓ All 5 loops start processing
```

#### 2️⃣ **Backend Processes Data (5 Concurrent Loops)**

##### **Loop 1: Detection Loop (3-5s interval)**

```
ESP32-CAM sends camera frame via WebSocket
    ↓
Backend receives JPEG bytes
    ↓
OpenCV decodes: cv2.imdecode(bytes) → numpy array
    ↓
YOLO11n processes image
    ↓
Detects: [{"label": "cup", "confidence": 0.85, "bbox": [100, 150, 80, 120]}]
    ↓
Gets depth from fusion loop
    ↓
Stores: DetectedObject(label="cup", depth=2.3m, position=(2.3, 1.8))
```

##### **Loop 2: Fusion Loop (500ms interval)**

```
iPhone sends LiDAR point cloud via WebSocket
{
  "points": [
    {"x": 0.5, "y": 0.2, "z": 1.2},
    {"x": 1.0, "y": 0.3, "z": 2.3},
    ...1000+ points
  ]
}
    ↓
Backend processes:
1. Project 3D points → 2D occupancy grid (100x100)
   → [0=free space, 255=obstacle]
2. Project 3D points → Camera frame using OpenCV
   → Creates aligned depth map (640x480)
3. Use cv2.inpaint() to fill gaps
    ↓
Stores:
• occupancy_grid (for D* Lite pathfinding)
• depth_map (for YOLO depth estimation)
```

##### **Loop 3: Planning Loop (1s interval)**

```
Get detected objects from state_manager
    ↓
Ask Gemini 2.0 Flash ADK agent:
"I see a cup at 2.3m, 45° right.
 Mission: Find the red mug.
 Should I navigate to it?"
    ↓
Gemini analyzes and responds:
{
  "action": "navigate_to_object",
  "target": "cup",
  "reasoning": "Cup detected with high confidence,
                investigate to verify if it's the target"
}
    ↓
Sets goal: state_manager.goal_position = (2.3, 1.8)
    ↓
Updates status: path_status = "PLANNING"
```

##### **Loop 4: Path Planning Loop (1s interval)**

```
Get occupancy grid from fusion loop
    ↓
Get goal position from planning loop
    ↓
Get current robot position from odometry
    ↓
Run D* Lite algorithm:
1. Initialize grid with costs
2. Find optimal path avoiding obstacles
3. Generate waypoint sequence
    ↓
Path found: [(0,0) → (0.5,0.3) → (1.0,0.8) → (2.3,1.8)]
    ↓
Next waypoint: (0.5, 0.3)
    ↓
Updates:
• state_manager.current_path = [...]
• state_manager.next_waypoint = (0.5, 0.3)
• path_status = "PATH_FOUND"
```

##### **Loop 5: Motor Control Loop (3s interval)**

```
Get next waypoint: (0.5, 0.3)
    ↓
Get robot position from odometry: (0, 0, heading=0°)
    ↓
Calculate required movement:
• Distance to waypoint: 0.58 meters
• Angle to waypoint: 31°
• Required turn: 31° right
    ↓
Translate to motor speeds:
• Left motor: 180 (slower to turn right)
• Right motor: 220 (faster to turn right)
    ↓
Create JSON command:
{
  "N": 6,         // Direct motor control
  "H": "cmd_42",  // Command ID
  "D1": 180,      // Left motor speed
  "D2": 220       // Right motor speed
}
    ↓
Send via WebSocket to ESP32 → /ws/arduino
```

#### 3️⃣ **Command Flows to Hardware**

```
Backend sends JSON via WebSocket
    ↓
ESP32 WiFi Bridge receives:
{
  "N": 6,
  "H": "cmd_42",
  "D1": 180,
  "D2": 220
}
    ↓
ESP32 forwards to Arduino via Serial (TX2→RX)
    ↓
Arduino Uno receives and parses JSON
    ↓
Extracts: commandType=6, leftSpeed=180, rightSpeed=220
    ↓
Calls: setMotorSpeeds(180, 220)
    ↓
Controls L298N motor driver:
• Left motor: digitalWrite(IN1, HIGH), analogWrite(ENA, 180)
• Right motor: digitalWrite(IN3, HIGH), analogWrite(ENB, 220)
    ↓
L298N outputs PWM to motors
    ↓
🚗 CAR TURNS RIGHT AND MOVES FORWARD!
```

#### 4️⃣ **Feedback Loop (Continuous)**

```
Arduino reads MPU6050 IMU every 100ms
    ↓
Current heading: 31° (turned right as expected!)
    ↓
Creates JSON: {"yaw": 31.0, "pitch": 0.2, "roll": -0.1}
    ↓
Sends to ESP32 via Serial (TX→RX2)
    ↓
ESP32 forwards to backend via WebSocket
    ↓
Backend receives IMU data
    ↓
Odometry updates robot position:
• Old: (0, 0, heading=0°)
• New: (0.1, 0.05, heading=31°)
    ↓
Path planning recalculates:
"Still on track, continue to next waypoint"
    ↓
CONTINUOUS NAVIGATION UNTIL TARGET REACHED!
```

#### 5️⃣ **Frontend Updates (Every 500ms)**

```
Frontend polls: GET /api/mission_status
    ↓
Backend responds:
{
  "status": "ACTIVE",
  "progress": 65,
  "current_action": "Navigating to target",
  "completed": false
}
    ↓
Frontend updates UI:
• Activity Log: ✓ "Navigating to target"
• Progress Bar: 65%
• Status: ACTIVE

Frontend polls: GET /api/detections
    ↓
Backend responds:
{
  "objects": [
    {"label": "cup", "confidence": 0.85, "bbox": [100,150,80,120], "depth": 2.3}
  ]
}
    ↓
Frontend canvas draws:
• Green bounding box at [100, 150, 80, 120]
• Label: "cup 85% 2.3m"
    ↓
USER SEES LIVE VISUALIZATION!
```

---

## 📡 API Documentation

### REST Endpoints

#### `GET /api/status`
**Purpose**: Get complete system status

**Response**:
```json
{
  "server": "running",
  "yolo_loaded": true,
  "adk_agent_initialized": true,
  "loops": {
    "detection": {"running": true, "total_runs": 42},
    "planning": {"running": true},
    "motor_control": {"running": true},
    "path_planning": {"running": true},
    "fusion": {"running": true, "total_fusions": 128}
  },
  "mission": {
    "objective": "Find the red mug",
    "mission_active": true,
    "elapsed_time_seconds": 23.5
  }
}
```

#### `POST /api/start_mission`
**Purpose**: Start autonomous navigation

**Request**:
```json
{
  "mission": "Find the red mug",
  "timestamp": "2025-10-26T12:00:00Z"
}
```

**Response**:
```json
{
  "status": "started",
  "objective": "Find the red mug"
}
```

#### `POST /api/stop_mission`
**Purpose**: Emergency stop

**Response**:
```json
{
  "status": "stopped",
  "message": "Mission stopped successfully"
}
```

#### `GET /api/mission_status`
**Purpose**: Real-time mission progress (polled by frontend every 500ms)

**Response**:
```json
{
  "status": "ACTIVE",
  "progress": 65,
  "current_action": "Navigating to target",
  "completed": false,
  "objective": "Find the red mug"
}
```

#### `GET /api/detections`
**Purpose**: Get YOLO-detected objects with depth

**Response**:
```json
{
  "count": 3,
  "objects": [
    {
      "label": "cup",
      "confidence": 0.85,
      "bbox": [100, 150, 80, 120],
      "depth": 2.3,
      "timestamp": 1730000000.0
    }
  ]
}
```

### WebSocket Endpoints

#### `WS /ws/lidar`
**Purpose**: Receive iPhone LiDAR point cloud

**Client sends**:
```json
{
  "timestamp": 1730000000.0,
  "points": [
    {"x": 0.5, "y": 0.2, "z": 1.2},
    {"x": 1.0, "y": 0.3, "z": 2.3}
  ]
}
```

#### `WS /ws/camera`
**Purpose**: Receive ESP32 camera frames

**Client sends**: Raw JPEG bytes (binary)

#### `WS /ws/arduino`
**Purpose**: Bidirectional Arduino communication

**Server sends (motor command)**:
```json
{
  "N": 6,
  "H": "cmd_42",
  "D1": 180,
  "D2": 220
}
```

**Client sends (IMU data)**:
```json
{
  "type": "imu",
  "yaw": 45.2,
  "pitch": 0.1,
  "roll": -0.5
}
```

---

## 🐛 Troubleshooting

### Backend Issues

**Problem**: `ModuleNotFoundError: No module named 'fastapi'`
```bash
# Solution: Install dependencies
pip install -r requirements.txt
```

**Problem**: `GOOGLE_API_KEY not set`
```bash
# Solution: Set environment variable
export GOOGLE_API_KEY="your-key-here"
```

**Problem**: YOLO model download fails
```bash
# Solution: Manually download
python -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"
```

### Frontend Issues

**Problem**: "Backend connection failed"
- Check backend is running: `python main.py`
- Verify IP address is correct
- Check firewall settings (allow port 8000)

**Problem**: Video stream not showing
- ESP32-CAM might not be connected
- Check WebSocket connection in browser console
- Verify camera is streaming via `/ws/camera`

### Arduino Issues

**Problem**: ESP32 won't connect to WiFi
- Check SSID/password are correct
- WiFi must be 2.4GHz (ESP32 doesn't support 5GHz)
- Move closer to router

**Problem**: Arduino upload fails
- Disconnect ESP32 TX/RX from Arduino Pin 0/1
- Select correct board: "Arduino Uno"
- Try different USB cable

**Problem**: Motors don't move
- Check battery voltage (7-12V)
- Verify L298N wiring
- Check if motor driver is overheating

---

## 🏛️ Architecture Details

### Technology Stack

**Backend:**
- **FastAPI** - Async web framework
- **YOLO11n** - Object detection (Ultralytics)
- **Google Gemini 2.0 Flash** - AI planning (ADK)
- **OpenCV** - Image processing & sensor fusion
- **D\* Lite** - Dynamic pathfinding algorithm
- **WebSockets** - Real-time communication

**Frontend:**
- **React** (via CDN) - UI framework
- **Canvas API** - Bounding box overlay
- **Fetch API** - Backend polling

**Hardware:**
- **ELEGOO Smart Car V4.0** - Robot platform
- **Arduino Uno** - Motor control
- **ESP32** - WiFi bridge
- **MPU6050** - IMU (gyro/accelerometer)
- **L298N** - Motor driver
- **iPhone LiDAR** - 3D environment scanning
- **ESP32-CAM** - Vision

### Key Algorithms

1. **YOLO11n** - Object detection with 80+ classes
2. **D\* Lite** - Incremental pathfinding with replanning
3. **Sensor Fusion** - LiDAR + Camera alignment via OpenCV
4. **Complementary Filter** - IMU noise reduction
5. **Occupancy Grid** - 2D environment representation

### Performance Metrics

| Component | Frequency | Latency |
|-----------|-----------|---------|
| YOLO Detection | 0.2-0.3 Hz | ~200ms |
| Gemini Planning | 1 Hz | ~500ms |
| Sensor Fusion | 2 Hz | ~10ms |
| Path Planning | 1 Hz | ~50ms |
| Motor Control | 0.3 Hz | ~5ms |
| **Total Loop Time** | **30 Hz** | **~800ms** |

---

## 🤝 Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Open a Pull Request

---

## 📄 License

MIT License - see LICENSE file for details

---

## 🙏 Acknowledgments

- **ELEGOO** - Robot car platform
- **Ultralytics** - YOLO implementation
- **Google** - Gemini AI
- **OpenCV** - Computer vision library

---

## 📞 Support

**Issues?** Open a GitHub issue or contact the maintainers.

**Documentation:**
- Backend API: `http://localhost:8000/docs` (when running)
- Hardware Setup: `backend/arduino/HARDWARE_SETUP_GUIDE.md`
- Code Audit: `backend/CODE_AUDIT.md`

---

## 🎉 Ready to Go!

Your autonomous navigation system is complete!

```bash
# Start backend
cd backend && python main.py

# Open frontend
open ../frontenddashboard.html

# Upload Arduino code
# (See Arduino Setup section)

# Give it a mission!
"Find the red coffee mug" 🚗💨
```

**Happy exploring!** 🤖✨
