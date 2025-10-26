# COMPLETE SERVER WORKFLOW - AUTONOMOUS RC CAR NAVIGATION

**Generated:** 2025-10-25 (UPDATED)
**Status:** Phase 2 (50% complete)
**Architecture:** FastAPI + ADK + YOLO11 + D* Lite

---

## HARDWARE SETUP

| Component | Specification | Interface |
|-----------|--------------|-----------|
| **RC Car** | ELEGOO Smart Robot Car V4.0 | Arduino Uno R3 |
| **Motor Driver** | L298N Dual H-Bridge | PWM (Pins 5, 6) + Direction (Pins 7, 8) |
| **IMU** | MPU6050 6-axis (Gyro+Accel) | I2C (built-in, calibrated) |
| **Ultrasonic** | HC-SR04 (2-400cm range) | Trigger/Echo pins |
| **IR Sensors** | ITR20001 x3 (L/M/R) | Analog pins (ground detection) |
| **Camera** | ESP32-CAM (2MP) | WiFi WebSocket |
| **LiDAR** | iPhone LiDAR (360° depth) | WiFi WebSocket |
| **RGB LEDs** | WS2812B x5 (NeoPixel) | Pin 3 |
| **Communication** | Serial UART (9600 baud) | USB/Bluetooth |
| **Pathfinding** | D* Lite algorithm | Incremental replanning |

### ELEGOO Motor Control Details (CRITICAL - Hardware Specific)

**⚠️ IMPORTANT: Motor B has INVERTED direction logic!**

- **Right Motor (A)**: PWM Pin 5, Direction Pin 8
  - `AIN_1 = LOW` → Forward
  - `AIN_1 = HIGH` → Backward

- **Left Motor (B)**: PWM Pin 6, Direction Pin 7
  - `BIN_1 = HIGH` → Forward (INVERTED!)
  - `BIN_1 = LOW` → Backward (INVERTED!)

- **Serial Protocol**: JSON over UART at **9600 baud** (confirmed in Arduino code)
- **Command Format**: `{"N":4, "H":"001", "D1":200, "D2":200}`
  - `N=4`: Direct motor speed control (best for autonomous navigation)
  - `H`: Command serial number (3 digits, e.g., "001", "042")
  - `D1`: Left motor speed (0-255)
  - `D2`: Right motor speed (0-255)

- **Response Format**:
  - Success: `{CMD_001_ok}`
  - Failure: `{CMD_001_false}`

### Available Sensors (Hardware Capabilities)

| Sensor | Purpose | Arduino Command | Server Integration |
|--------|---------|----------------|-------------------|
| **MPU6050 IMU** | Yaw angle for heading | Custom command needed | ❌ TODO - Add endpoint |
| **ITR20001 IR (x3)** | Ground detection | `{"N":23, "H":"GND"}` | ❌ TODO - Safety feature |
| **HC-SR04 Ultrasonic** | Distance measurement | `{"N":21, "H":"DST", "D1":1/2}` | ⚠️ Optional (LiDAR preferred) |

---

## SERVER WORKFLOW - STEP BY STEP

## 1. INITIALIZATION

### 1.1 Load YOLO model into memory ✅ **DONE**
- **Model:** YOLO11n (nano, 1.5ms inference)
- **Confidence threshold:** 0.7
- **Implementation:** `models/yolo_model.py`
- **Status:** Fully functional

### 1.2 Initialize Gemini API client (ADK) ✅ **DONE**
- **Model:** gemini-2.0-flash-exp
- **Agent type:** Navigation agent with custom tools
- **Implementation:** `agents/navigation_agent.py`
- **Tools provided:**
  - `calculate_distance_to_object`
  - `check_path_clear`
  - `describe_environment`
  - `check_goal_reached`
  - `validate_path_safety`
  - `calculate_motor_command_for_waypoint`
  - `get_exploration_command`

### 1.3 Create shared state manager ✅ **DONE**
- **Implementation:** `state_manager.py`
- **Features:**
  - Thread-safe in-memory storage
  - Tracks: detections, positions, sensor data
  - D* Lite integration structures prepared
  - Motor command queue
  - Odometry tracking fields

### 1.4 Initialize D* Lite pathplanner ❌ **TODO - CRITICAL**

**Source Location:** `../Dstar-lite-pathplanner/python/python/`

**Required Files to Copy:**
```
backend/pathfinding/
├── __init__.py
├── d_star_lite.py          # Main D* Lite algorithm
├── grid.py                 # OccupancyGridMap class
├── priority_queue.py       # Priority queue for D* Lite
└── utils.py                # Heuristic functions (Euclidean distance)
```

**Implementation Tasks:**
- [ ] Copy 4 files from `../Dstar-lite-pathplanner/python/python/` to `backend/pathfinding/`
- [ ] Create wrapper class `DStarLitePlanner` in `backend/pathfinding/planner_wrapper.py`
- [ ] Initialize occupancy grid map (from LiDAR)
- [ ] Set start position: (0, 0)
- [ ] Set goal position: (unknown until object detected)
- [ ] Configure grid resolution: 10cm cells
- [ ] Movement mode: 8-connectivity (allows diagonal movement)

**Wrapper Class Design:**
```python
# backend/pathfinding/planner_wrapper.py
from .d_star_lite import DStarLite
from .grid import OccupancyGridMap
import numpy as np

class DStarLitePlanner:
    """
    Wrapper for D* Lite pathplanner integrating with server state
    """
    def __init__(self, grid_size=(200, 200), resolution=0.1):
        """
        Args:
            grid_size: (x_dim, y_dim) in grid cells
            resolution: meters per cell (0.1 = 10cm cells)
        """
        self.resolution = resolution
        self.grid_size = grid_size

        # Create occupancy grid (8-connectivity for diagonal moves)
        self.map = OccupancyGridMap(
            x_dim=grid_size[0],
            y_dim=grid_size[1],
            exploration_setting='8N'
        )

        # Initialize D* Lite (start at origin, goal unknown)
        self.planner = None  # Created when goal is set
        self.current_position = (0, 0)
        self.goal_position = None

    def set_goal(self, world_x: float, world_y: float):
        """
        Set goal position when target object detected
        Args:
            world_x, world_y: Goal position in meters
        """
        goal_grid = self._world_to_grid(world_x, world_y)
        self.goal_position = goal_grid

        # Initialize planner with current position and goal
        self.planner = DStarLite(
            map=self.map,
            s_start=self.current_position,
            s_goal=goal_grid
        )

    def update_occupancy_grid(self, lidar_points: list):
        """
        Update occupancy grid from LiDAR point cloud
        Args:
            lidar_points: List of (x, y, z) tuples in meters
        """
        # Convert LiDAR points to 2D occupancy grid
        for point in lidar_points:
            x, y, z = point
            # Ignore points too high (not obstacles) or too low (floor)
            if 0.05 < z < 1.5:  # Between 5cm and 1.5m height
                grid_pos = self._world_to_grid(x, y)
                if self._in_bounds(grid_pos):
                    self.map.set_obstacle(grid_pos)

    def plan_path(self, current_world_x: float, current_world_y: float):
        """
        Plan path from current position to goal
        Returns:
            path: List of (x, y) waypoints in world coordinates (meters)
            or None if no path exists
        """
        if self.planner is None:
            return None

        current_grid = self._world_to_grid(current_world_x, current_world_y)
        self.current_position = current_grid

        try:
            # D* Lite incremental replanning
            path_grid, g, rhs = self.planner.move_and_replan(current_grid)

            # Convert grid path to world coordinates
            path_world = [self._grid_to_world(x, y) for (x, y) in path_grid]
            return path_world

        except AssertionError as e:
            # No path exists
            return None

    def _world_to_grid(self, x: float, y: float) -> tuple:
        """Convert world coordinates (meters) to grid coordinates"""
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        return (grid_x, grid_y)

    def _grid_to_world(self, grid_x: int, grid_y: int) -> tuple:
        """Convert grid coordinates to world coordinates (meters)"""
        world_x = grid_x * self.resolution
        world_y = grid_y * self.resolution
        return (world_x, world_y)

    def _in_bounds(self, grid_pos: tuple) -> bool:
        """Check if grid position is within bounds"""
        x, y = grid_pos
        return 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]
```

**D* Lite Algorithm Notes:**
- Uses **Euclidean heuristic** (from `utils.py`)
- Supports **8-connectivity** movement (diagonal moves cost √2)
- **Incremental replanning**: Only recomputes affected path sections when obstacles change
- **Priority queue** based on lexicographic key ordering

### 1.5 Set mission objective ✅ **DONE**
- **Default:** "Find a red coffee mug"
- **Implementation:** `config.py` + `state_manager.py`
- **Source:** Frontend via REST API

### 1.6 Initialize Frontend API Endpoints ✅ **DONE**

**REST API for Frontend Integration:**

| Method | Endpoint | Purpose | Payload | Response |
|--------|----------|---------|---------|----------|
| POST | `/api/mission/start` | Start new mission | `{"objective": "find X"}` | `{"status": "mission_started"}` |
| POST | `/api/mission/stop` | Stop current mission | None | `{"status": "stopped"}` |
| GET | `/api/mission/status` | Get mission status | None | `{"status": "searching", "objective": "..."}` |
| POST | `/api/emergency_stop` | Emergency motor stop | None | `{"status": "stopped"}` |
| WS | `/ws/mission_updates` | Real-time updates | N/A | Stream of detections/position |

**Implementation:** `main.py:194-200` (existing) + new endpoints needed

### 1.7 Start WebSocket listeners ⚠️ **PARTIAL**

- ✅ **ESP32-CAM** (camera stream) - `ws://localhost:8002`
  - Endpoint: `/ws/camera` in `main.py:334`
  - Accepts: JPEG compressed images
  - Validation: Size checks, JPEG header validation

- ✅ **iPhone LiDAR** (depth map) - `ws://localhost:8001`
  - Endpoint: `/ws/lidar` in `main.py:244`
  - Accepts: `{"points": [{"x": ..., "y": ..., "z": ...}]}`
  - Validation: Point cloud structure validation

- ❌ **Arduino motor control** (bidirectional) - **NEW IMPLEMENTATION NEEDED**

  **Recommended Approach: Serial over USB (PySerial)**
  ```python
  # communication/elegoo_serial_client.py
  import serial
  import json
  import time

  class ELEGOOSerialClient:
      def __init__(self, port='/dev/ttyUSB0', baud=9600, timeout=0.5):
          """
          BAUD RATE: 9600 (confirmed in SmartRobotCarV4.0.ino line 97)
          """
          self.ser = serial.Serial(port, baud, timeout=timeout)
          self.command_counter = 0
          time.sleep(2)  # Wait for Arduino to reset after serial connection

      def send_motor_command(self, left_speed: int, right_speed: int) -> bool:
          """
          Send motor speed command (N=4)
          Args:
              left_speed: 0-255 (D1)
              right_speed: 0-255 (D2)
          Returns:
              True if acknowledged, False otherwise
          """
          self.command_counter += 1
          command = {
              "N": 4,
              "H": f"{self.command_counter:03d}",
              "D1": left_speed,
              "D2": right_speed
          }

          # Send command (add closing brace)
          self.ser.write((json.dumps(command) + '}').encode())

          # Wait for response: {CMD_001_ok} or {CMD_001_false}
          response = self.ser.readline().decode().strip()
          return "_ok}" in response

      def request_imu_yaw(self) -> float:
          """
          Request current yaw angle from MPU6050 IMU
          Note: Requires custom Arduino command implementation
          """
          # TODO: Add custom command {"N":24, "H":"YAW"}
          pass

      def check_ground_contact(self) -> bool:
          """
          Check if car is on the ground (ITR20001 sensors)
          Command: {"N":23, "H":"GND"}
          Response: {CMD_GND_true} (on ground) or {CMD_GND_false} (lifted)
          """
          command = {"N": 23, "H": "GND"}
          self.ser.write((json.dumps(command) + '}').encode())

          response = self.ser.readline().decode().strip()
          return "_true}" in response
  ```

### 1.8 Print: "Server ready" ✅ **DONE**
- **Implementation:** `main.py:112-116`

---

## 2. CONTINUOUS SENSOR INPUT LOOP (1-2 Hz)

### 2.1 Receive iPhone LiDAR data ✅ **DONE**
- **Format:** 360° depth map + point cloud
- **Validation:** Timestamp validation
- **Implementation:** `main.py:244-332` (WebSocket handler)
- **Storage:** `state_manager.update_lidar(data, timestamp)`

### 2.2 Receive ESP32-CAM frame ✅ **DONE**
- **Format:** 2MP RGB image, JPEG compressed
- **Implementation:** `main.py:334-400` (WebSocket handler)
- **Storage:** `state_manager.update_camera(frame, timestamp)`

### 2.3 Check data freshness ✅ **DONE**
- **Timeout:** 2000ms
- **Validation:** Both sensor timestamps checked
- **Implementation:** `state_manager.has_fresh_sensor_data()`

### 2.4 If stale → skip iteration
- **Implementation:** `detection_loop.py:98-104`

### 2.5 If fresh → proceed to fusion

---

## 3. SENSOR FUSION (<100ms)

### 3.1 Align LiDAR to camera ❌ **TODO**
- **Requirements:**
  - Camera intrinsics matrix (focal length, principal point)
  - LiDAR-to-camera extrinsic calibration
  - Perspective transformation
- **Current state:** Heuristic alignment in `detection_loop.py:192-322`

### 3.2 Create 3D point cloud ❌ **TODO**
- **Output:** RGB color labels from camera + XYZ coordinates from LiDAR
- **Format:** `[(x, y, z, r, g, b), ...]`

### 3.3 Convert to occupancy grid ❌ **CRITICAL**

**Implementation needed:** `fusion/occupancy_grid_generator.py`

```python
import numpy as np

def lidar_to_occupancy_grid(points, resolution=0.1, grid_size=(200, 200)):
    """
    Convert LiDAR point cloud to 2D occupancy grid

    Args:
        points: List of (x, y, z) tuples in meters
        resolution: Grid cell size in meters (0.1 = 10cm)
        grid_size: (x_dim, y_dim) number of cells

    Returns:
        occupancy_grid: numpy array (grid_size) where:
            0 = free space
            255 = obstacle
            1-254 = uncertainty/partial occupancy
    """
    grid = np.zeros(grid_size, dtype=np.uint8)

    for (x, y, z) in points:
        # Filter by height (ignore floor and ceiling)
        if not (0.05 < z < 1.5):  # Between 5cm and 1.5m
            continue

        # Convert world coords to grid coords
        grid_x = int(x / resolution)
        grid_y = int(y / resolution)

        # Check bounds
        if 0 <= grid_x < grid_size[0] and 0 <= grid_y < grid_size[1]:
            grid[grid_x, grid_y] = 255  # Mark as obstacle

    # Apply dilation for safety margin (expand obstacles by 1 cell)
    from scipy.ndimage import binary_dilation
    obstacle_mask = grid > 0
    dilated = binary_dilation(obstacle_mask, iterations=1)
    grid[dilated] = 255

    return grid
```

**Integration with D* Lite:**
- **Format:** 2D numpy array (compatible with `OccupancyGridMap`)
- **Resolution:** 10cm cells (configurable)
- **Values:** `0` = free, `255` = obstacle (matches D* Lite `OBSTACLE` constant)

### 3.4 Update D* Lite map ❌ **CRITICAL**

**Implementation:** `loops/path_planning_loop.py`

```python
async def path_planning_loop(state_manager, planner: DStarLitePlanner):
    """
    Continuously update path when occupancy grid changes
    Runs at 1 Hz OR when new obstacles detected
    """
    while not state_manager.mission_complete:
        # Get latest occupancy grid
        occupancy_grid = state_manager.get_occupancy_grid()

        if occupancy_grid is not None:
            # Update D* Lite map with new obstacles
            # This triggers incremental replanning if path is affected
            planner.update_occupancy_grid(occupancy_grid)

            # Replan path if goal is set
            if planner.goal_position is not None:
                current_pos = state_manager.get_current_position()
                path = planner.plan_path(current_pos.x, current_pos.y)

                if path:
                    state_manager.update_path(path, "PATH_FOUND")
                else:
                    state_manager.update_path(None, "NO_PATH")

        await asyncio.sleep(1.0)  # 1 Hz update rate
```

**Trigger:** Incremental update when new obstacles detected
**Action:** Triggers replanning if current path affected

### 3.5 Store fused data ❌ **TODO**
- **Storage:** `state_manager.update_occupancy_grid(grid, timestamp)`
- **Fields available:** Already prepared in `state_manager.py:66-71`

---

## 4. OBJECT DETECTION (2-5 seconds)

### 4.1 Pull latest RGB frame ✅ **DONE**
- **Implementation:** `detection_loop.py:89`

### 4.2 Run YOLO ✅ **DONE**
- **Implementation:** `detection_loop.py:119-124`
- **Async execution:** Runs in thread pool executor to avoid blocking

### 4.3 Get detection results ✅ **DONE**
- **Format:**
  ```python
  [
    {
      'label': 'mug',
      'confidence': 0.92,
      'bbox': [x, y, w, h],
      'depth': 2.3  # meters
    }
  ]
  ```

### 4.4 Filter by confidence ✅ **DONE**
- **Threshold:** >0.7
- **Implementation:** `detection_loop.py:132-136`

### 4.5 Enrich with LiDAR depth ⚠️ **PLACEHOLDER**
- **Current:** Heuristic estimation in `detection_loop.py:192-322`
- **Needs:** Proper sensor fusion (see Section 3.1)
- **Method:**
  - Project LiDAR points to camera frame
  - Filter points within bounding box
  - Use median depth for robustness

### 4.6 Store detections ✅ **DONE**
- **Implementation:** `detection_loop.py:169-174`
- **Storage:** `state_manager.add_detected_object(obj)`

### 4.7 IF target object found ❌ **TODO**
- **Tasks:**
  - [ ] Calculate 3D world position from depth + bbox center
  - [ ] Convert world position to grid coordinates
  - [ ] Set D* Lite goal position: `planner.set_goal(world_x, world_y)`
  - [ ] Update path status: `state_manager.path_status = "PLANNING"`

**Implementation:**
```python
# In detection_loop.py after object detection
if target_found:
    # Get object position in camera frame
    bbox_center_x = bbox[0] + bbox[2] / 2
    bbox_center_y = bbox[1] + bbox[3] / 2
    depth = object_depth  # From LiDAR fusion

    # Convert to world coordinates (assuming camera at origin)
    world_x = depth  # Forward direction
    world_y = (bbox_center_x - image_width/2) * depth / focal_length

    # Set as goal for pathplanner
    planner.set_goal(world_x, world_y)
    state_manager.update_path_status("PLANNING")
```

---

## 5. PLANNING/REASONING LOOP (1 Hz)

### **Stage 1: Path Planning (D* Lite)** ❌ **TODO**

#### 5.1 Check if goal is set
- **Condition:** Has target object been detected?
- **Implementation:** Check `planner.goal_position is not None`

#### 5.2 Get current RC car position
- **Source:** Odometry/tracking with **MPU6050 IMU**
- **Current:** Placeholder at (0, 0, 0)
- **Implementation needed:** Integrate IMU yaw + dead reckoning

**IMPORTANT: ELEGOO has MPU6050 6-axis IMU (Gyro + Accelerometer)**
- Already calibrated in `MPU6050_calibration()` during standby mode
- Used in `ApplicationFunctionSet_SmartRobotCarLinearMotionControl()` for straight-line correction
- Provides accurate **yaw (heading) angle** for direction tracking

**Recommended Position Tracking:**
```python
# tools/odometry.py
import math

class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0  # radians
        self.last_update = time.time()

    def update_from_motor_commands(self, left_speed: int, right_speed: int,
                                   imu_yaw: float = None):
        """
        Update position using dead reckoning + IMU

        Args:
            left_speed: Left motor PWM (0-255)
            right_speed: Right motor PWM (0-255)
            imu_yaw: Yaw angle from MPU6050 in degrees (if available)
        """
        dt = time.time() - self.last_update
        self.last_update = time.time()

        # Constants (calibrate experimentally)
        MAX_SPEED_M_S = 0.5  # Max speed in m/s at PWM=255
        WHEEL_BASE = 0.15    # Distance between wheels in meters

        # Calculate linear and angular velocity
        v_left = (left_speed / 255.0) * MAX_SPEED_M_S
        v_right = (right_speed / 255.0) * MAX_SPEED_M_S
        v_linear = (v_left + v_right) / 2.0

        if imu_yaw is not None:
            # Use IMU heading (more accurate!)
            self.heading = math.radians(imu_yaw)
        else:
            # Estimate heading from differential drive
            v_angular = (v_right - v_left) / WHEEL_BASE
            self.heading += v_angular * dt

        # Update position
        self.x += v_linear * math.cos(self.heading) * dt
        self.y += v_linear * math.sin(self.heading) * dt

    def get_position(self):
        return (self.x, self.y, self.heading)
```

#### 5.3 Run D* Lite incremental pathplanning ❌ **CRITICAL**

```python
# In path_planning_loop.py
current_pos = odometry.get_position()
path = planner.plan_path(current_pos[0], current_pos[1])

if path:
    state_manager.update_path(path, "PATH_FOUND")
else:
    state_manager.update_path(None, "NO_PATH")
```

- **Output:** List of waypoints `[(x1, y1), (x2, y2), ...]` in world coordinates (meters)
- **Storage:** `state_manager.update_path(path, "PATH_FOUND")`

#### 5.4 Get next waypoint from path
- **Implementation:** `state_manager.next_waypoint` (automatically set when path updated)
- **Format:** `(x, y)` in meters (world coordinates)

#### 5.5 If no path exists → Exploration mode
- **Fallback:** Systematic rotation and scanning
- **Strategy:** Rotate 360° in 45° increments, scanning for target at each stop

---

### **Stage 2: High-Level Decision (ADK Agent)** ⚠️ **MODIFIED**

#### 5.6 Retrieve latest state ✅ **DONE**
- **Current data:**
  - ✅ Current objective
  - ✅ All detected objects + locations
- **Missing data:**
  - ❌ Current path from D* Lite
  - ❌ Current waypoint
  - ❌ Obstacles detected
- **Implementation:** `planning_loop.py:88` + `agents/navigation_agent.py:170`

**UPDATE NEEDED:** Enhance `prepare_agent_state()` to include path info

#### 5.7 Build context prompt for ADK ⚠️ **NEEDS UPDATE**

**Current:** Basic object list
**Needed:** Include path planner status, waypoint, obstacles

```python
# agents/navigation_agent.py (update prepare_agent_state)
def prepare_agent_state(state_manager):
    return {
        "objective": state_manager.objective,
        "detected_objects": state_manager.get_detected_objects(),
        "current_position": state_manager.get_current_position(),
        "path_status": state_manager.path_status,  # NEW
        "current_path": state_manager.current_path,  # NEW
        "next_waypoint": state_manager.next_waypoint,  # NEW
        "obstacles_detected": state_manager.get_obstacles(),  # NEW
    }
```

#### 5.8 ADK Agent decides ❌ **TODO**
- **Decision tree:**
  1. **Follow D* Lite path?** (normal case)
  2. **Override for safety?** (unexpected obstacle)
  3. **Switch to exploration?** (no goal set)
  4. **Emergency stop?** (sensor failure)

#### 5.9 Generate motor command ❌ **CRITICAL CHANGE**

**Current output:** Text instruction for human
**Needed output:** Motor command dict

**Target internal format:**
```python
{
  "command": "MOVE_TO_WAYPOINT",
  "waypoint": (x, y),
  "speed": 50,  # 0-100%
  "avoid_obstacles": True
}
```

**Translation to ELEGOO Serial format:**

```python
# tools/motor_command_translator.py
import math

def translate_to_elegoo_command(waypoint: tuple, current_pos: tuple,
                                current_heading: float, speed_percent: int,
                                command_counter: int) -> dict:
    """
    Convert high-level waypoint command to ELEGOO motor speeds

    Args:
        waypoint: (x, y) target position in meters
        current_pos: (x, y) current position in meters
        current_heading: Current heading in radians
        speed_percent: Desired speed 0-100%
        command_counter: Sequential command number

    Returns:
        ELEGOO command dict: {"N":4, "H":"001", "D1":200, "D2":200}
    """
    # Calculate heading to waypoint
    dx = waypoint[0] - current_pos[0]
    dy = waypoint[1] - current_pos[1]
    target_heading = math.atan2(dy, dx)

    # Calculate heading error
    heading_error = target_heading - current_heading
    # Normalize to [-pi, pi]
    heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

    # Convert speed percentage to PWM (0-255)
    base_speed = int((speed_percent / 100.0) * 255)

    # Proportional turning control
    Kp = 100  # Proportional gain (tune experimentally)
    turn_adjustment = int(Kp * heading_error)

    # Calculate differential drive speeds
    left_speed = base_speed - turn_adjustment
    right_speed = base_speed + turn_adjustment

    # Clamp to valid range [0, 255]
    left_speed = max(0, min(255, left_speed))
    right_speed = max(0, min(255, right_speed))

    return {
        "N": 4,
        "H": f"{command_counter:03d}",
        "D1": left_speed,   # Left motor
        "D2": right_speed   # Right motor
    }
```

#### 5.10 Store command in output queue ✅ **DONE**
- **Implementation:** `state_manager.add_motor_command(command)`
- **Storage:** `state_manager.motor_command_queue`

---

## 6. DECISION LOGIC (after each planning cycle)

### 6.1 Check if objective found ✅ **PARTIAL**
- **Checks:**
  - ✅ Is target in detected_objects?
  - ✅ Confidence > 0.95?
  - ❌ Distance < 0.5m? (needs proper depth estimation)
- **Implementation:** `planning_loop.py:125-180`

### 6.2 IF YES → Step 8 (SUCCESS)
- **Implementation:** `planning_loop.py:68-84`

### 6.3 IF NO but object detected ❌ **TODO**
- **Action:** Trigger D* Lite path to object
- **Implementation:** Call `planner.set_goal(object_x, object_y)`

### 6.4 IF object not visible ❌ **TODO**
- **Action:** Enter exploration mode
  - Systematic search pattern
  - Rotate 45° and scan
  - Move forward if clear

### 6.5 IF STUCK ❌ **TODO**
- **Detection:** Velocity ~0 for 5+ seconds with non-zero motor command
- **Action:**
  - Backup 0.3m
  - Find alternative route
  - Re-run D* Lite

---

## 7. MOTOR CONTROL OUTPUT LOOP (0.3 Hz)

🔄 **COMPLETELY NEW** - replaces TTS audio output

### 7.1 Check command queue ❌ **TODO**
- **Implementation needed:** `loops/motor_control_loop.py`
- **Check:** `state_manager.pop_motor_command()`

### 7.2 IF command exists:

#### 7.2.1 Parse command to motor values ❌ **TODO**

**Translation layer:** Use `motor_command_translator.py` (see Section 5.9)

```python
# High-level command from ADK
{
  "command": "MOVE_TO_WAYPOINT",
  "waypoint": (2.3, 4.1),
  "speed": 60
}

# Convert to ELEGOO Serial JSON
{
  "N": 4,
  "H": f"{command_counter:03d}",
  "D1": left_motor_pwm,   # Calculated from heading error
  "D2": right_motor_pwm   # Calculated from heading error
}
```

#### 7.2.2 Send to Arduino via Serial ❌ **CRITICAL**

**Implementation: Serial over USB (PySerial) - Recommended for MVP**

```python
# loops/motor_control_loop.py
import asyncio
from communication.elegoo_serial_client import ELEGOOSerialClient

async def motor_control_loop(state_manager, odometry):
    """
    Motor control loop: Execute motor commands and update odometry
    Runs at 0.3 Hz (every 3 seconds + execution time)
    """
    # Initialize serial connection
    arduino = ELEGOOSerialClient(port='/dev/ttyUSB0', baud=9600)

    while not state_manager.mission_complete:
        # Get next command from queue
        command = state_manager.pop_motor_command()

        if command:
            # Translate high-level command to motor speeds
            motor_cmd = translate_to_elegoo_command(
                waypoint=command['waypoint'],
                current_pos=odometry.get_position()[:2],
                current_heading=odometry.get_position()[2],
                speed_percent=command['speed'],
                command_counter=arduino.command_counter
            )

            # Send to Arduino
            success = arduino.send_motor_command(
                motor_cmd['D1'],
                motor_cmd['D2']
            )

            if success:
                # Update odometry
                odometry.update_from_motor_commands(
                    motor_cmd['D1'],
                    motor_cmd['D2']
                )

                # Update state manager
                state_manager.update_position(odometry.get_position())
            else:
                # Retry logic
                pass
        else:
            # No command: keep motors stopped
            arduino.send_motor_command(0, 0)

        await asyncio.sleep(3.0)  # 0.3 Hz
```

**BAUD RATE: 9600** (confirmed in `SmartRobotCarV4.0.ino` line 97)

**Important Notes:**
- Serial connection auto-resets Arduino (2-second delay needed)
- Ensure Arduino firmware uses `Serial.begin(9600)`
- USB port typically `/dev/ttyUSB0` (Linux) or `/dev/cu.usbserial-*` (Mac)

#### 7.2.3 Log command ❌ **TODO**
- **Format:** `[MOTOR] CMD#042: {"N":4, "D1":200, "D2":200}`
- **Implementation:** Add logging in `motor_control_loop.py`

#### 7.2.4 Wait for Arduino confirmation ❌ **TODO**
- **Expected response:** `{CMD_001_ok}`
- **Timeout:** 500ms
- **Retry logic:** 3 attempts

**Implementation:**
```python
def send_motor_command_with_retry(arduino, left, right, max_retries=3):
    for attempt in range(max_retries):
        success = arduino.send_motor_command(left, right)
        if success:
            return True
        time.sleep(0.1)  # Small delay before retry
    return False
```

#### 7.2.5 Update RC car position ❌ **TODO - USE MPU6050 IMU!**

**HARDWARE AVAILABLE:** ELEGOO has MPU6050 6-axis IMU!

**Recommended Approach:**
1. **Primary**: MPU6050 yaw + wheel speed estimation
2. **Fallback**: Pure dead reckoning

**Implementation:**
```python
# Request yaw from Arduino (REQUIRES CUSTOM COMMAND)
# Add to Arduino firmware: {"N":24, "H":"YAW"} → {YAW_123.45}

imu_yaw = arduino.request_imu_yaw()  # Returns yaw in degrees

# Update odometry with IMU heading
odometry.update_from_motor_commands(
    left_speed=motor_cmd['D1'],
    right_speed=motor_cmd['D2'],
    imu_yaw=imu_yaw  # Use actual IMU reading!
)

new_position = odometry.get_position()  # (x, y, heading)
state_manager.update_position(new_position)
```

**Dead Reckoning Formula (if IMU unavailable):**
```python
dt = time_elapsed
avg_speed = (left_motor + right_motor) / 2 * MAX_SPEED_M_S / 255
heading_change = (right_motor - left_motor) * TURN_FACTOR

new_x = old_x + avg_speed * dt * cos(heading)
new_y = old_y + avg_speed * dt * sin(heading)
new_heading = old_heading + heading_change * dt
```

#### 7.2.6 Update D* Lite with new position ❌ **TODO**
- **Trigger:** Replanning if position significantly changed (>10cm)
- **Implementation:** Already handled in `path_planning_loop.py` (Section 3.4)

### 7.3 IF NO command:
- **Action:** Keep motors stopped
- **Implementation:** Send `{"N":4, "H":"000", "D1":0, "D2":0}`
- **Wait:** 100ms
- **Safety:** Ensure stop command sent if no activity for >2s

---

## 8. SUCCESS STATE

### 8.1 Detect success ❌ **TODO**
- **Criteria:**
  - Target in view
  - Confidence > 0.95
  - Distance < 0.5m (needs proper depth estimation)
- **Implementation:** Part of `planning_loop.py:125-180`

### 8.2 Stop planning loop ❌ **TODO**
- **Implementation:** `planning_loop.py:54` already checks `mission_complete`
- **Action:** Set `state_manager.mission_complete = True`

### 8.3 Send STOP command to RC car ❌ **TODO**
```json
{"N":4, "H":"999", "D1":0, "D2":0}
```

### 8.4 Signal success ❌ **TODO**

**Arduino LED/buzzer:** Send RGB LED command

```json
{"N":8, "H":"LED", "D1":0, "D2":255, "D3":0, "D4":0}
```
- `D1=0`: All LEDs
- `D2=0, D3=255, D4=0`: Green color (success!)

**LED Control Notes (from ELEGOO code):**
- ELEGOO has **5x WS2812B NeoPixel LEDs** on Pin 3
- LED positions: 0=back, 1=right, 2=front, 3=left, 4=center
- Command N=8: Control individual or all LEDs

### 8.5 Freeze sensor streams ❌ **TODO**
- **Action:** Stop WebSocket loops (optional, or just pause processing)
- **Implementation:** Set flag to pause `detection_loop.py` and `path_planning_loop.py`

### 8.6 Log mission success ✅ **PARTIAL**
- **Current logging:**
  - ✅ Time taken
  - ✅ Basic stats
- **Missing:**
  - ❌ Motor commands sent count
  - ❌ Path taken visualization
  - ❌ D* Lite replanning count

**Enhanced Logging:**
```python
mission_stats = {
    "objective": state_manager.objective,
    "time_elapsed": time.time() - mission_start_time,
    "motor_commands_sent": arduino.command_counter,
    "path_length_meters": calculate_path_length(state_manager.current_path),
    "replanning_count": planner.replan_counter,
    "final_distance_to_target": final_distance,
    "final_confidence": final_confidence
}
```

### 8.7 Print to console ✅ **PARTIAL**
```
✅ MISSION COMPLETE: Red mug found!
Distance: 0.3m | Confidence: 0.97 | Commands: 42 | Replans: 5 | Time: 3m 24s
```

### 8.8 Wait for new mission ✅ **DONE**
- **Implementation:** `main.py:194-200` (POST `/api/mission/start`)
- **Action:** Reset state and wait for frontend to send new objective

---

## 9. ERROR HANDLING

### 9.1 Sensor data missing >2s ⚠️ **PARTIAL**

✅ **Log warning** - `detection_loop.py:102`
❌ **STOP RC car motors immediately** - **CRITICAL SAFETY**

**Implementation needed:**
```python
# In motor_control_loop.py
if time.time() - last_sensor_update > 2.0:
    # Emergency stop!
    arduino.send_motor_command(0, 0)
    logger.critical("EMERGENCY STOP: Sensor data timeout!")
    state_manager.path_status = "SENSOR_FAILURE"
```

**BONUS SAFETY - Ground Detection Available!**

ELEGOO has **ITR20001 infrared sensors** (3x) that detect if car leaves the ground:
- Arduino checks: `ApplicationFunctionSet_SmartRobotCarLeaveTheGround()`
- Prevents motors from running if car is lifted

**Safety Protocol:**
```python
# Request ground status
ground_contact = arduino.check_ground_contact()

if not ground_contact:
    # Car is lifted! Stop motors
    arduino.send_motor_command(0, 0)
    logger.warning("Car lifted off ground - motors stopped")
    await asyncio.sleep(1.0)
    continue
```

**Implementation:**
- [ ] Add ground check command: `{"N":23, "H":"GND"}`
- [ ] Response: `{CMD_GND_true}` (on ground) or `{CMD_GND_false}` (lifted)
- [ ] Check before each motor command
- [ ] STOP motors if lifted for >1 second

### 9.2 Pathfinding fails ❌ **TODO**

**Fallback actions:**
1. Try simpler path (increase grid resolution)
2. Switch to exploration mode
3. Request human intervention via frontend

**Implementation:**
```python
if path is None:
    logger.warning("No path found to target")
    state_manager.path_status = "NO_PATH"

    # Fallback: Switch to exploration mode
    enter_exploration_mode(state_manager)
```

### 9.3 YOLO crashes ⚠️ **PARTIAL**

✅ **Logged** - Exception handling in `detection_loop.py:184`
❌ **Fallback to LiDAR-only** - Not implemented

**Recommendation:** Continue with exploration mode if YOLO fails

### 9.4 WiFi disconnected ⚠️ **PARTIAL**

✅ **Detected** - WebSocket disconnect handlers
❌ **STOP motors immediately** - **CRITICAL SAFETY**

**Implementation:**
```python
# In WebSocket disconnect handler
@app.websocket("/ws/lidar")
async def lidar_disconnect():
    logger.critical("LiDAR disconnected - stopping motors")
    arduino.send_motor_command(0, 0)
    state_manager.path_status = "LIDAR_DISCONNECTED"
```

### 9.5 RC car stuck ❌ **NEW**

**Detection:** Compare commanded velocity vs actual movement

**Implementation:**
```python
# In motor_control_loop.py
if motor_cmd['D1'] > 50 or motor_cmd['D2'] > 50:
    # Motors are running
    position_change = calculate_distance(old_pos, new_pos)

    if position_change < 0.05:  # Less than 5cm movement
        stuck_counter += 1

        if stuck_counter > 5:  # 5 iterations ~15 seconds
            logger.warning("Car appears stuck - executing recovery")

            # Recovery maneuver: backup and turn
            arduino.send_motor_command(150, 150)  # Backward
            await asyncio.sleep(1.0)

            arduino.send_motor_command(200, 100)  # Turn right
            await asyncio.sleep(0.5)

            # Replan path
            planner.current_position = odometry.get_position()[:2]
            stuck_counter = 0
```

### 9.6 Arduino Communication Failure ❌ **NEW**

**Detection:** No acknowledgment from Arduino after 3 retries

**Implementation:**
```python
if not send_motor_command_with_retry(arduino, left, right):
    logger.error("Arduino communication failed")

    # Try to reconnect
    try:
        arduino.ser.close()
        arduino = ELEGOOSerialClient(port='/dev/ttyUSB0', baud=9600)
        logger.info("Arduino reconnected")
    except Exception as e:
        logger.critical(f"Cannot reconnect to Arduino: {e}")
        state_manager.mission_complete = True
```

---

## 10. FRONTEND-BACKEND INTEGRATION

### Frontend Responsibilities:

1. **Mission Start Interface**
   - Input field: "What should the car find?" (e.g., "red coffee mug")
   - Submit button → `POST /api/mission/start`
   - **Payload:** `{"objective": "find a red coffee mug"}`

2. **Mission Monitoring Dashboard** (Optional)
   - Display current objective
   - Show detection results (objects found)
   - Display mission status (searching/approaching/complete)
   - Real-time position visualization
   - WebSocket connection for live updates

3. **Emergency Stop Button**
   - Large red button
   - Send `POST /api/emergency_stop` → Car immediately stops

### Backend Endpoints for Frontend:

```python
# Mission control
POST   /api/mission/start        # Start new mission with objective
POST   /api/mission/stop         # Stop current mission
GET    /api/mission/status       # Get current mission status

# Real-time updates (WebSocket)
WS     /ws/mission_updates       # Stream: detections, position, path

# Emergency
POST   /api/emergency_stop       # Immediate motor stop + reset
```

### Data Flow:

```
┌─────────┐                  ┌─────────┐                  ┌─────────┐
│Frontend │──{"objective"}──>│ Backend │──motor cmds────>│ Arduino │
│         │<──progress────────│         │<──ack───────────│         │
└─────────┘   (WebSocket)    └─────────┘    (Serial)     └─────────┘
                                   │
                                   ├──>YOLO11
                                   ├──>D* Lite
                                   └──>ADK Agent
```

### WebSocket Message Format:

```json
{
  "type": "detection",
  "timestamp": 1234567890.123,
  "data": {
    "label": "mug",
    "confidence": 0.92,
    "position": {"x": 2.3, "y": 1.1}
  }
}
```

```json
{
  "type": "position_update",
  "timestamp": 1234567890.456,
  "data": {
    "x": 1.5,
    "y": 0.8,
    "heading": 45.2
  }
}
```

```json
{
  "type": "path_update",
  "timestamp": 1234567890.789,
  "data": {
    "path": [[0,0], [1,1], [2,3], [2.3,4.1]],
    "status": "PATH_FOUND"
  }
}
```

---

## 11. ASYNC CONCURRENCY

All loops running simultaneously:

```
├─[Sensor Input]───────────────  every 500ms
│  └─ WebSocket handlers (main.py) ✅
│
├─[Fusion + OccGrid]───────────  every 500ms (triggered by input)
│  └─ TODO: Create fusion_loop.py ❌
│
├─[YOLO Detection]─────────────  every 3-5 seconds
│  └─ loops/detection_loop.py ✅
│
├─[D* Lite Planning]───────────  every 1s OR when map changes
│  └─ TODO: Create path_planning_loop.py ❌
│
├─[ADK Decision]───────────────  every 1 second
│  └─ loops/planning_loop.py ✅
│
├─[Motor Control]──────────────  every 3s + wait for execution
│  └─ TODO: Create motor_control_loop.py ❌
│
├─[Odometry Update]────────────  after each motor command
│  └─ TODO: Integrate in motor_control_loop.py ❌
│
└─[Safety Monitor]─────────────  every 500ms
   └─ TODO: Create safety_watchdog.py ❌
      ├─ Check sensor freshness
      ├─ Check ground contact
      └─ Check Arduino connection
```

---

## IMPLEMENTATION STATUS

### ✅ **COMPLETE**
- [x] YOLO model loading (`models/yolo_model.py`)
- [x] ADK agent with tools (`agents/navigation_agent.py`)
- [x] State manager (`state_manager.py`)
- [x] WebSocket endpoints for sensors (`main.py`)
- [x] Detection loop - YOLO processing (`loops/detection_loop.py`)
- [x] Planning loop - ADK agent (`loops/planning_loop.py`)
- [x] Basic error handling
- [x] Frontend API endpoints (`/api/mission/start`)

### ⚠️ **PARTIAL**
- [ ] Sensor fusion (placeholder depth estimation)
- [ ] Decision logic (basic implementation, needs path integration)
- [ ] Error handling (needs motor safety stops)
- [ ] Mission success detection (distance check incomplete)
- [ ] ADK context (needs path/obstacle info)

### ❌ **TODO (CRITICAL)**

#### **Priority 1 - Critical for MVP (Days 1-2)**

1. **D* Lite integration** 🔴 **BLOCKING**
   - [ ] Copy 4 files from `../Dstar-lite-pathplanner/python/python/`:
     - `d_star_lite.py`
     - `grid.py`
     - `priority_queue.py`
     - `utils.py`
   - [ ] Create `pathfinding/planner_wrapper.py` (wrapper class)
   - [ ] Integrate with occupancy grid
   - [ ] Test with mock obstacles

2. **Occupancy grid generation** 🔴 **BLOCKING**
   - [ ] Create `fusion/occupancy_grid_generator.py`
   - [ ] Convert LiDAR point cloud to 2D grid
   - [ ] Filter by height (0.05m - 1.5m)
   - [ ] Apply safety dilation (1 cell)
   - [ ] Update on sensor fusion loop

3. **Motor control loop** 🔴 **BLOCKING**
   - [ ] Create `loops/motor_control_loop.py`
   - [ ] Implement command queue processing
   - [ ] Create `communication/elegoo_serial_client.py`
   - [ ] Test Serial communication (9600 baud, USB)
   - [ ] Add retry logic (3 attempts)
   - [ ] Implement emergency stop logic

4. **Arduino communication** 🔴 **BLOCKING**
   - [ ] Set up Serial connection (PySerial, 9600 baud)
   - [ ] Test bidirectional protocol
   - [ ] Add connection health monitoring
   - [ ] Parse acknowledgments (`{CMD_001_ok}`)
   - [ ] Test with real ELEGOO hardware

#### **Priority 2 - Enhanced functionality (Days 3-4)**

5. **Proper sensor fusion**
   - [ ] Camera-LiDAR calibration
   - [ ] 3D point cloud with RGB
   - [ ] Accurate depth estimation for bounding boxes

6. **Position tracking/odometry** 🟡 **IMPORTANT**
   - [ ] Create `tools/odometry.py`
   - [ ] Implement dead reckoning
   - [ ] **Add custom Arduino command for MPU6050 yaw** (recommended)
   - [ ] Update D* Lite with new positions
   - [ ] Test position accuracy

7. **Path planning loop** 🟡 **IMPORTANT**
   - [ ] Create `loops/path_planning_loop.py`
   - [ ] Integrate D* Lite replanning
   - [ ] Trigger on obstacle detection
   - [ ] Update waypoints in state manager

8. **Safety features** 🟡 **IMPORTANT**
   - [ ] Create `safety_watchdog.py`
   - [ ] Emergency stop on sensor timeout
   - [ ] Ground detection integration (ITR20001)
   - [ ] Stuck detection and recovery
   - [ ] Arduino connection monitoring

#### **Priority 3 - Polish (Days 5-6)**

9. **Exploration mode**
   - [ ] Systematic search pattern
   - [ ] Rotation and scanning (45° increments)
   - [ ] Frontier-based exploration (optional)

10. **Performance optimization**
    - [ ] Profile loop timing
    - [ ] Optimize occupancy grid updates
    - [ ] Reduce latency (<500ms sensor-to-motor)

11. **Error recovery**
    - [ ] Automatic Arduino reconnection
    - [ ] State recovery after crash
    - [ ] Graceful degradation

12. **Logging and monitoring**
    - [ ] Structured logging (JSON format)
    - [ ] Performance metrics dashboard
    - [ ] Mission replay capability (path visualization)

13. **Testing with real hardware**
    - [ ] ELEGOO car Serial integration
    - [ ] ESP32-CAM WiFi setup
    - [ ] iPhone LiDAR app connection
    - [ ] End-to-end mission test
    - [ ] Calibration (motor speeds, turning radius)

---

## KEY ARCHITECTURAL DECISIONS

### Original Misunderstanding → Corrected Implementation

| Original (Wrong) | Corrected (Actual) |
|------------------|-------------------|
| Human with blindfold | RC Car autonomous |
| Audio TTS instructions | Motor control commands (JSON/Serial) |
| Human moves manually | Car moves autonomously |
| Simple navigation | D* Lite pathfinding |
| Wait for human confirmation | Wait for motor execution |
| Update human position | Update robot odometry (MPU6050 IMU!) |

---

## COMMUNICATION PROTOCOLS

### ELEGOO Arduino Serial Protocol (COMPLETE)

**Baud Rate:** 9600 (confirmed in `SmartRobotCarV4.0.ino:97`)

**Motor Control Commands:**

```json
// N=4: Direct motor speed (RECOMMENDED for autonomous)
{"N":4, "H":"001", "D1":200, "D2":200}
// D1: Left motor (0-255)
// D2: Right motor (0-255)

// N=3: Direction control (alternative)
{"N":3, "H":"002", "D1":1, "D2":150}
// D1: 1=Forward, 2=Back, 3=Left, 4=Right, 5-8=Diagonal, 9=Stop
// D2: Speed (0-255)
```

**Sensor Query Commands:**

```json
// N=21: Ultrasonic distance
{"N":21, "H":"DST", "D1":1}
// D1: 1=status (obstacle?), 2=distance (cm)
// Response: {CMD_DST_true/false} or {CMD_DST_45}

// N=22: IR tracking sensors
{"N":22, "H":"TRK", "D1":0}
// D1: 0=left, 1=middle, 2=right
// Response: {CMD_TRK_123} (analog value)

// N=23: Ground contact check (ITR20001 sensors)
{"N":23, "H":"GND"}
// Response: {CMD_GND_true} (on ground) or {CMD_GND_false} (lifted)
```

**LED Control Commands:**

```json
// N=8: RGB LED control
{"N":8, "H":"LED", "D1":0, "D2":255, "D3":0, "D4":0}
// D1: LED index (0=back, 1=right, 2=front, 3=left, 4=center, 5=all)
// D2: Red (0-255)
// D3: Green (0-255)
// D4: Blue (0-255)
```

**Response Format:**
```
{CMD_001_ok}    // Success
{CMD_001_false} // Failure
```

### Custom Commands to Add (Arduino firmware modification needed)

```json
// Recommended: MPU6050 yaw angle request
{"N":24, "H":"YAW"}
// Response: {YAW_123.45} (degrees)
```

### WebSocket Protocols

**LiDAR (ws://localhost:8001)**
```json
{
  "timestamp": 1234567890.123,
  "points": [
    {"x": 1.2, "y": 0.5, "z": 2.3},
    {"x": 1.3, "y": 0.6, "z": 2.4}
  ]
}
```

**Camera (ws://localhost:8002)**
- Binary JPEG data
- No JSON wrapper
- Size: 100 bytes - 10MB

**Mission Updates (ws://localhost:8000/ws/mission_updates)** - NEW
```json
{
  "type": "detection|position|path|status",
  "timestamp": 1234567890.123,
  "data": { ... }
}
```

---

## FILE STRUCTURE (UPDATED)

```
backend/
├── main.py                          ✅ FastAPI app + WebSocket endpoints
├── config.py                        ✅ Configuration settings
├── state_manager.py                 ✅ Thread-safe shared state
├── adk_session_manager.py           ✅ ADK session management
│
├── agents/
│   ├── __init__.py
│   └── navigation_agent.py          ✅ ADK agent definition
│
├── models/
│   ├── __init__.py
│   └── yolo_model.py                ✅ YOLO wrapper
│
├── loops/
│   ├── __init__.py
│   ├── detection_loop.py            ✅ YOLO object detection
│   ├── planning_loop.py             ✅ ADK decision making
│   ├── motor_control_loop.py        ❌ TODO - Motor command execution
│   ├── path_planning_loop.py        ❌ TODO - D* Lite pathfinding
│   ├── fusion_loop.py               ❌ TODO - Sensor fusion
│   └── safety_watchdog.py           ❌ TODO - Safety monitoring
│
├── tools/
│   ├── __init__.py
│   ├── navigation_tools.py          ✅ ADK tool functions
│   ├── motor_command_translator.py  ❌ TODO - Waypoint → Motor speeds
│   └── odometry.py                  ❌ TODO - Position tracking
│
├── pathfinding/                     ❌ TODO - Create this directory
│   ├── __init__.py
│   ├── d_star_lite.py               ❌ COPY from ../Dstar-lite-pathplanner/
│   ├── grid.py                      ❌ COPY from ../Dstar-lite-pathplanner/
│   ├── priority_queue.py            ❌ COPY from ../Dstar-lite-pathplanner/
│   ├── utils.py                     ❌ COPY from ../Dstar-lite-pathplanner/
│   └── planner_wrapper.py           ❌ TODO - D* Lite wrapper class
│
├── fusion/                          ❌ TODO - Create this directory
│   ├── __init__.py
│   ├── occupancy_grid_generator.py  ❌ TODO - LiDAR → Grid conversion
│   └── sensor_calibration.py        ❌ TODO - Camera-LiDAR alignment
│
└── communication/                   ❌ TODO - Create this directory
    ├── __init__.py
    ├── elegoo_serial_client.py      ❌ TODO - Serial communication class
    └── elegoo_wifi_bridge.py        ❌ FUTURE - WiFi bridge (optional)
```

---

## NEXT IMMEDIATE STEPS

### Day 1 (Today) - D* Lite + Occupancy Grid 🔴

1. **Copy D* Lite files**
   ```bash
   mkdir -p backend/pathfinding
   cp ../Dstar-lite-pathplanner/python/python/*.py backend/pathfinding/
   ```

2. **Create planner wrapper** (`pathfinding/planner_wrapper.py`)
3. **Create occupancy grid generator** (`fusion/occupancy_grid_generator.py`)
4. **Test pathfinding** with mock occupancy grid

### Day 2 - Motor Control 🔴

5. **Create motor control loop** (`loops/motor_control_loop.py`)
6. **Implement Serial client** (`communication/elegoo_serial_client.py`)
7. **Test with ELEGOO car** (wired USB connection)
8. **Create motor command translator** (`tools/motor_command_translator.py`)

### Day 3 - Integration

9. **Create path planning loop** (`loops/path_planning_loop.py`)
10. **Create odometry tracker** (`tools/odometry.py`)
11. **Integrate D* Lite with motor control**
12. **Add sensor fusion loop** (`loops/fusion_loop.py`)

### Day 4 - Safety & Testing

13. **Implement safety watchdog** (`loops/safety_watchdog.py`)
14. **Add ground detection check**
15. **Add stuck recovery**
16. **End-to-end testing** with real hardware

### Day 5 - Polish

17. **Add exploration mode**
18. **Optimize performance**
19. **Add detailed logging**
20. **Frontend integration testing**

---

## ESTIMATED TIMELINE

| Phase | Duration | Status |
|-------|----------|--------|
| **Foundation** (YOLO, ADK, State) | 2 days | ✅ **COMPLETE** |
| **D* Lite Integration** | 1 day | 🔴 **TODO** |
| **Motor Control** | 1 day | 🔴 **TODO** |
| **Sensor Fusion** | 1 day | ⚠️ **PARTIAL** |
| **Integration & Testing** | 1 day | 🔴 **TODO** |
| **Safety & Polish** | 1 day | 🔴 **TODO** |
| **Total** | **7 days** | **~45% complete** |

---

## TESTING CHECKLIST

### Hardware Tests
- [ ] ELEGOO receives Serial commands at 9600 baud
- [ ] Arduino acknowledges commands (`{CMD_001_ok}`)
- [ ] Motors respond to PWM values (0-255)
- [ ] Motor B inverted direction confirmed
- [ ] Ground detection (ITR20001) working
- [ ] RGB LEDs controllable via Serial

### Software Tests
- [ ] D* Lite generates valid path on test grid
- [ ] Occupancy grid converts LiDAR correctly
- [ ] YOLO detects target object with >0.7 confidence
- [ ] LiDAR provides depth data for detections
- [ ] Motor commands translate correctly to ELEGOO format
- [ ] Odometry tracks position with <10% error
- [ ] Path replanning triggers on obstacle detection
- [ ] Emergency stop works on sensor timeout
- [ ] Frontend can start/stop missions

### Integration Tests
- [ ] End-to-end: Frontend → Detection → Path → Motors
- [ ] Mission completes successfully when target reached
- [ ] Safety watchdog stops car when lifted
- [ ] Stuck recovery executes backup maneuver
- [ ] Multiple missions can run sequentially

---

## HARDWARE CALIBRATION NOTES

### Motor Speed Calibration
```python
# Experimentally determine these values:
MAX_SPEED_M_S = 0.5      # Maximum speed at PWM=255 (meters/second)
WHEEL_BASE = 0.15        # Distance between wheels (meters)
TURN_FACTOR = 1.0        # Calibration for turning rate
```

**Calibration Procedure:**
1. Command car forward at PWM=255 for 10 seconds
2. Measure distance traveled → `MAX_SPEED_M_S = distance / 10`
3. Command differential speeds (left=255, right=128) for 10 seconds
4. Measure rotation angle → adjust `TURN_FACTOR`

### IMU Calibration
- ELEGOO already calibrates MPU6050 during standby mode
- Calibration runs in `MPU6050_calibration()` (ensure car is on flat surface)
- Yaw angle is used for straight-line correction

---

**End of Server Workflow Documentation**

---

## CHANGELOG

**2025-10-25 - MAJOR UPDATE**
- ✅ Added complete D* Lite integration guide with wrapper class
- ✅ Discovered ELEGOO has MPU6050 IMU - updated odometry section
- ✅ Discovered ground detection sensors (ITR20001) - added safety feature
- ✅ Confirmed Serial baud rate: 9600 (from Arduino source code)
- ✅ Documented motor direction inversion (Motor B)
- ✅ Added complete Arduino Serial protocol reference
- ✅ Added frontend-backend integration section
- ✅ Added occupancy grid generation algorithm
- ✅ Added motor command translation logic
- ✅ Expanded file structure with all required modules
- ✅ Updated timeline and testing checklist
- ✅ Added hardware calibration procedures
