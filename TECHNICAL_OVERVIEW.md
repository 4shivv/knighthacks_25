# Technical Overview: Autonomous Navigation System

## Human-in-the-Loop vs Human-out-of-the-Loop

### Human-in-the-Loop (HITL) Approach
In HITL systems, humans actively participate in decision-making during operation:
- **Supervision**: Operator monitors and intervenes when needed
- **Feedback**: Real-time adjustments based on human judgment
- **Safety**: Human can override autonomous decisions
- **Use Case**: Critical missions requiring human oversight

### Human-out-of-the-Loop (HOOTL) Approach - Our Implementation
Our system implements HOOTL autonomous navigation where the robot operates independently:

**Mission Start (Human Input)**:
```
User: "Find the red mug"  → System activates
```

**Autonomous Operation Phase (No Human)**:
- Robot scans environment with LiDAR + Camera
- YOLO AI detects objects autonomously
- Google Gemini ADK makes navigation decisions
- D* Lite plans paths around obstacles
- Motor control executes movements
- Continuous replanning as environment changes

**Human Returns for Results**:
```
System: "Target found at (2.3m, 1.8m)" → Human reviews outcome
```

**Key Advantage**: The system operates continuously without human intervention, making real-time decisions based on sensor data and AI reasoning.

---

## Google ADK (Agent Development Kit) Architecture

### What is Google ADK?
Google's Agent Development Kit enables building stateful AI agents using Gemini models with tool access and session management.

### Our ADK Implementation

#### 1. Session Management
**Location**: `backend/adk_session_manager.py`

```python
class ADKSessionManager:
    def __init__(self, agent, app_name):
        # In-memory session service for real-time performance
        self.session_service = InMemorySessionService()

        # Runner executes agent with tool access
        self.runner = Runner(
            agent=self.agent,
            app_name=self.app_name,
            session_service=self.session_service
        )
```

**Key Features**:
- **InMemorySessionService**: Fast, ephemeral state storage for real-time navigation
- **Session State**: Maintains navigation context across planning cycles
- **Event History**: Tracks AI decisions and tool calls

#### 2. Navigation Agent Definition
**Location**: `backend/agents/navigation_agent.py`

```python
navigation_agent = LlmAgent(
    name="navigation_agent",
    model='gemini-2.0-flash-exp',

    # Dynamic system instruction with state templating
    instruction="""You are an autonomous navigation decision system for an RC car.

**Current Mission:** {objective}

**Your Role:**
- Monitor D* Lite pathplanner status and make high-level navigation decisions
- Override pathplanner when safety requires immediate action
- Generate structured motor commands or path-following directives
- Decide between: following planned path, emergency stop, exploration, or replanning

**Current Status:**
- Position: {current_position}
- Heading: {current_heading}
- Velocity: {velocity}
- Detected objects: {detected_objects_summary}
- Path status: {path_status}
- Next waypoint: {next_waypoint}
- Goal position: {goal_position}
- Path length: {current_path_length} steps
- Obstacles in path: {obstacles_in_path}
- Commands sent: {motor_commands_sent}
- D* Lite replans: {dstar_replans}

**Decision Logic (choose ONE):**
1. **MISSION_COMPLETE** - Target found and reached
2. **FOLLOW_PATH** - Trust D* Lite pathplanner
3. **EMERGENCY_STOP** - Immediate safety concern
4. **REQUEST_REPLAN** - Path invalidated by new obstacles
5. **EXPLORE** - No goal set yet, systematic search
6. **BACKUP_RECOVERY** - RC car stuck or no progress

**Output Format:**
Return ONLY a valid JSON motor command object.

Example: {{"command": "MOVE_TO_WAYPOINT", "waypoint": [2.3, 4.1], "speed": 60}}""",

    # Navigation tools for Gemini to call
    tools=[
        calculate_distance_to_object,
        check_path_clear,
        describe_environment,
        check_goal_reached,
        validate_path_safety,
        calculate_motor_command_for_waypoint,
        get_exploration_command
    ],

    # Callbacks for validation and monitoring
    before_agent_callback=before_agent_callback,  # Validates fresh sensor data
    after_agent_callback=after_agent_callback,    # Logs decisions

    output_key="last_instruction"  # Save response to session state
)
```

**ADK Agent Callbacks**:
```python
async def before_agent_callback(**kwargs):
    """
    Validates sensor data freshness before agent runs
    - Checks LiDAR timestamp < 2 seconds old
    - Checks camera timestamp < 2 seconds old
    - Returns error if data stale
    """
    has_lidar = context.state.get('temp:last_lidar_timestamp', 0) > 0
    has_camera = context.state.get('temp:last_camera_timestamp', 0) > 0

    if not (has_lidar and has_camera):
        return "Waiting for sensor data..."

    return None  # Proceed with agent

async def after_agent_callback(**kwargs):
    """
    Logs agent decision and increments instruction counter
    - Extracts generated instruction from response
    - Updates state['total_instructions']
    """
    instruction = extract_text(context.response)
    logger.info(f"Generated instruction: {instruction}")
    context.state['total_instructions'] += 1
```

**Complete ADK Tool Workflow**:
```
Planning Loop (1 Hz)
    ↓
prepare_agent_state(state_manager) → Creates state dict:
{
  "objective": "Find red mug",
  "detected_objects_summary": "cup, chair",
  "current_position": "(0.1, 0.05, 0.0)",
  "path_status": "PATH_FOUND",
  "next_waypoint": "(2.3, 1.8)",
  "current_path_length": 15,
  "obstacles_in_path": 0,
  "motor_commands_sent": 42,
  "dstar_replans": 3,
  "temp:detected_objects": [  ← Raw data for tools
    {"label": "cup", "depth": 2.3, "confidence": 0.85}
  ]
}
    ↓
adk_session_manager.update_session_state(agent_state)
    ↓
adk_session_manager.run_agent("Generate next motor command")
    ↓
before_agent_callback() → Validates sensor data
    ↓
Gemini 2.0 Flash receives templated instruction:
"Current Mission: Find red mug
 Position: (0.1, 0.05, 0.0)
 Detected objects: cup, chair
 Path status: PATH_FOUND
 Next waypoint: (2.3, 1.8)
 ..."
    ↓
Gemini analyzes and calls tools:
1. calculate_distance_to_object("cup") → Returns 2.3m
2. validate_path_safety() → Returns True (no obstacles)
3. check_goal_reached() → Returns False (not at target yet)
    ↓
Gemini reasons:
"Cup detected at 2.3m with high confidence (0.85)
 Path to waypoint (2.3, 1.8) is clear
 D* Lite has valid path
 → Decision: FOLLOW_PATH"
    ↓
Gemini generates JSON response:
{
  "command": "MOVE_TO_WAYPOINT",
  "waypoint": [2.3, 1.8],
  "speed": 60,
  "reasoning": "Following D* Lite path to investigate cup"
}
    ↓
after_agent_callback() → Logs decision
    ↓
Planning Loop parses JSON motor command
    ↓
state_manager.add_motor_command(command)
    ↓
Motor Control Loop executes command → Drives robot
```

#### 3. State Preparation
```python
def prepare_agent_state(state_manager) -> Dict[str, Any]:
    """Convert StateManager to ADK-compatible state dict"""
    return {
        "detected_objects": [...],      # YOLO detections with depth
        "robot_position": (x, y),       # Odometry tracking
        "current_heading": 31.0,        # IMU data
        "path_status": "PLANNING",      # D* Lite status
        "occupancy_grid_summary": {...} # Obstacle map
    }
```

#### 4. Response Parsing & JSON Extraction
**Location**: `backend/loops/planning_loop.py:_parse_motor_command()`

The Planning Loop must extract structured JSON commands from Gemini's response:

```python
def _parse_motor_command(self, agent_response: str) -> Optional[dict]:
    """
    Parse motor command from agent response
    Handles multiple formats:
    1. Pure JSON: {"command": "MOVE_TO_WAYPOINT", ...}
    2. Markdown: ```json\n{...}\n```
    3. Text with embedded JSON: "Based on analysis... {"command": ...}"
    """
    # Try 1: Direct JSON parse
    try:
        return json.loads(agent_response)
    except json.JSONDecodeError:
        pass

    # Try 2: Extract from markdown code block
    if "```json" in agent_response:
        start = agent_response.find("```json") + 7
        end = agent_response.find("```", start)
        json_str = agent_response[start:end].strip()
        return json.loads(json_str)

    # Try 3: Regex pattern matching
    json_pattern = r'\{[^}]+\}'
    matches = re.findall(json_pattern, agent_response, re.DOTALL)
    for match in matches:
        try:
            parsed = json.loads(match)
            if 'command' in parsed:  # Validate it's a motor command
                return parsed
        except json.JSONDecodeError:
            continue

    return None  # Failed to parse
```

**Example Responses from Gemini**:
```
Response 1 (Pure JSON):
{"command": "MOVE_TO_WAYPOINT", "waypoint": [2.3, 1.8], "speed": 60}
→ Parses directly ✓

Response 2 (Markdown):
```json
{"command": "EMERGENCY_STOP", "reason": "Obstacle at 0.2m"}
```
→ Extracts from code block ✓

Response 3 (Text + JSON):
Based on the detected cup at 2.3m, I recommend investigating it.
{"command": "MOVE_TO_WAYPOINT", "waypoint": [2.3, 1.8], "speed": 60}
→ Regex extracts JSON ✓
```

**Why This Works**:
- **Stateful Reasoning**: ADK maintains context across planning cycles
- **Tool Access**: Gemini can query sensors and control navigation goals
- **Asynchronous**: `runner.run_async()` enables real-time decision-making
- **Robust Parsing**: Handles varied LLM response formats automatically

---

## D* Lite Pathfinding Algorithm

### What is D* Lite?
D* Lite (Dynamic A* Lite) is an incremental heuristic search algorithm that efficiently replans shortest paths when obstacles are discovered.

**Key Advantage over A***: Instead of replanning from scratch when obstacles appear, D* Lite reuses previous computations and only updates affected regions.

### Our D* Lite Implementation
**Location**: `backend/pathfinding/d_star_lite.py`

#### Core Algorithm Components

**1. Priority Queue with Key Calculation**
```python
def calculate_key(self, s: Tuple[int, int]) -> Priority:
    """
    Priority = [k1, k2] for lexicographic ordering
    k1 = min(g(s), rhs(s)) + h(s_start, s) + k_m
    k2 = min(g(s), rhs(s))
    """
    k1 = min(self.g[s], self.rhs[s]) + heuristic(s_start, s) + self.k_m
    k2 = min(self.g[s], self.rhs[s])
    return Priority(k1, k2)
```

**2. Cost Function**
```python
def c(self, u: Vertex, v: Vertex) -> float:
    """
    Edge cost between adjacent cells
    - Returns infinity if obstacle detected
    - Returns Euclidean distance if free space
    """
    if not self.map.is_unoccupied(u) or not self.map.is_unoccupied(v):
        return float('inf')
    return heuristic(u, v)  # Euclidean distance
```

**3. Main Search Loop**
```python
def compute_shortest_path(self):
    """
    D* Lite core algorithm - processes priority queue
    Updates g-values (cost-to-goal) and rhs-values (lookahead)
    """
    while U.top_key() < calculate_key(s_start) or rhs[s_start] > g[s_start]:
        u = U.top()  # Get vertex with lowest priority

        if g[u] > rhs[u]:  # Locally overconsistent
            g[u] = rhs[u]
            # Propagate to predecessors
            for s in predecessors(u):
                rhs[s] = min(rhs[s], c(s, u) + g[u])
                update_vertex(s)
        else:  # Locally underconsistent or consistent
            g[u] = infinity
            # Recalculate rhs for affected vertices
```

**4. Incremental Replanning**
```python
def update_map(self, changed_cells: List[Tuple[int, int]]):
    """
    Efficient replanning when obstacles discovered
    - Only recomputes affected regions
    - Updates k_m for consistent heuristic
    """
    self.k_m += heuristic(self.s_last, self.s_start)  # Maintain consistency

    for cell in changed_cells:
        # Update rhs values for neighbors
        for neighbor in neighbors(cell):
            recalculate_rhs(neighbor)
            update_vertex(neighbor)

    # Recompute only necessary vertices
    compute_shortest_path()
```

### Occupancy Grid Integration
**Location**: `backend/pathfinding/occupancy_grid.py`

```python
class OccupancyGrid:
    """
    2D grid map with obstacle detection
    - 0 = UNOCCUPIED (free space)
    - 255 = OBSTACLE (blocked)
    """

    def __init__(self, x_dim: int, y_dim: int, exploration_setting: str = '8N'):
        """
        exploration_setting:
        - '4N': 4-connectivity (only horizontal/vertical moves)
        - '8N': 8-connectivity (includes diagonals for smoother paths)
        """
        self.occupancy_grid_map = np.zeros((x_dim, y_dim), dtype=np.uint8)
        self.exploration_setting = exploration_setting

    def succ(self, vertex: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Get successor cells (neighbors) based on connectivity
        """
        if self.exploration_setting == '4N':
            # 4-connectivity: [(x±1, y), (x, y±1)]
            movements = get_movements_4n(x, y)  # 4 neighbors
        else:
            # 8-connectivity: [(x±1, y±1), (x±1, y), (x, y±1)]
            movements = get_movements_8n(x, y)  # 8 neighbors

        # Aesthetic path improvement: reverse movements for even cells
        if (x + y) % 2 == 0:
            movements.reverse()

        # Filter out obstacles and out-of-bounds
        return [m for m in movements if self.in_bounds(m) and self.is_unoccupied(m)]

    def update_from_sensor_data(self, occupancy_data: np.ndarray):
        """
        Update entire grid from fusion loop
        - Replaces grid with new sensor data
        - Used for incremental replanning
        """
        self.occupancy_grid_map = occupancy_data.astype(np.uint8)
```

**4N vs 8N Connectivity**:
```
4N (Manhattan distance):        8N (Euclidean distance):
      N                               NW  N  NE
      |                               \  |  /
  W---C---E                         W---C---E
      |                               /  |  \
      S                              SW  S  SE

Cost per move:                     Cost per move:
- Horizontal/Vertical: 1.0         - Horizontal/Vertical: 1.0
                                   - Diagonal: √2 ≈ 1.414

Path quality:                      Path quality:
- Jagged (90° turns only)         - Smooth (45° angles allowed)
- 4 successors per cell           - 8 successors per cell
- Faster computation               - Better paths, slightly slower
```

We use **8N connectivity** for smoother RC car paths.

### D* Lite Workflow in Our System

```
Path Planning Loop (1 Hz)
    ↓
Get occupancy grid from Fusion Loop
    ↓
Current Position: (0, 0) [from odometry]
Goal Position: (2.3, 1.8) [from Gemini ADK]
    ↓
Initialize D* Lite if first run:
  - Set s_start = current_grid(0, 0)
  - Set s_goal = goal_grid(23, 18)
  - Create OccupancyGrid with 8N connectivity
  - rhs[s_goal] = 0
  - g[all cells] = infinity
  - Insert s_goal into priority queue with key = h(start, goal)
  - compute_shortest_path()  → Full path computed via Dijkstra from goal
    ↓
Update occupancy grid from sensor fusion:
  - Get new grid from state_manager.occupancy_grid
  - Compare previous_grid vs new_grid
  - changed_mask = (previous != new)
  - Find changed cells: [(12, 8), (12, 9)] ← New obstacle detected!
    ↓
Incremental Replan (D* Lite magic):
  - update_map(changed_cells)
  - k_m += h(s_last, s_start)  ← Adjust heuristic for consistency
  - For each changed cell:
      * Get 8 neighbors (8N connectivity)
      * Recalculate rhs[neighbor] = min(c(neighbor, s') + g[s']) for all successors s'
      * update_vertex(neighbor) → Add to priority queue if inconsistent
  - compute_shortest_path()  → Only processes vertices in priority queue!
      * Expands ~10-50 cells instead of full 10,000 cell grid
      * Focuses on affected region near obstacle
    ↓
Get next waypoint:
  - get_next_waypoint(current_position)
  - Evaluate all 8 successors (8N)
  - Returns best: argmin[c(current, s) + g[s]]
  - Next waypoint: (0.5, 0.3)
    ↓
Convert grid → world coordinates:
  - world_x = grid_x * 0.1m
  - world_y = grid_y * 0.1m
    ↓
Update state_manager.next_waypoint = (0.5, 0.3)
    ↓
Motor Control Loop picks up waypoint and drives
```

**Performance Comparison**:
| Scenario | A* Full Replan | D* Lite Incremental |
|----------|----------------|---------------------|
| Initial path (100x100 grid) | 150-200ms | 150-200ms (same) |
| **5 cells change** | 150-200ms | **5-15ms** ⚡ |
| **20 cells change** | 150-200ms | **20-50ms** ⚡ |
| Memory usage | Low (one search) | Higher (maintains g & rhs arrays) |

**Why D* Lite is Faster**:
1. **Incremental**: Only updates affected regions
2. **k_m adjustment**: Maintains heuristic consistency without full recomputation
3. **Priority queue**: Only processes vertices that could improve the path
4. **Result**: 10-20x faster replanning for small obstacle changes

---

## Sensor Fusion Workflow

### What is Sensor Fusion?
Combining data from multiple sensors (LiDAR + Camera) to create a unified, more accurate representation of the environment.

### Our Sensor Fusion Implementation
**Location**: `backend/loops/fusion_loop.py`

#### Data Sources

**iPhone LiDAR** (WebSocket `/ws/lidar`):
```json
{
  "timestamp": 1730000000.0,
  "points": [
    {"x": 0.5, "y": 0.2, "z": 1.2},  // 3D point cloud
    {"x": 1.0, "y": 0.3, "z": 2.3},
    ...  // 1000+ points
  ]
}
```

**ESP32-CAM** (WebSocket `/ws/camera`):
- Raw JPEG bytes (640x480 RGB image)

#### Fusion Pipeline (Runs at 2 Hz)

**Step 1: Generate Occupancy Grid from LiDAR**
```python
def _generate_occupancy_grid(lidar_data) -> np.ndarray:
    """
    Project 3D LiDAR points → 2D occupancy grid for D* Lite

    Process:
    1. Filter points by height (0.05m < z < 0.5m) ← Ignore ground/ceiling
    2. Convert robot-frame → world coordinates
    3. Discretize to grid cells (0.1m resolution)
    4. Count points per cell
    5. Mark cell as obstacle if points >= threshold (5 points)

    Returns: 100x100 grid, 0=free space, 255=obstacle
    """
    grid = np.zeros((100, 100), dtype=np.uint8)
    point_counts = np.zeros((100, 100), dtype=np.int32)

    for point in lidar_data['points']:
        if min_height < point['z'] < max_height:  # Filter by height
            # World coordinates
            world_x = robot_x + point['x']
            world_y = robot_y + point['y']

            # Grid coordinates (robot at center)
            grid_x = int((world_x - origin_x) / 0.1)
            grid_y = int((world_y - origin_y) / 0.1)

            point_counts[grid_y, grid_x] += 1

    # Mark obstacles (≥5 points per cell)
    grid[point_counts >= 5] = 255

    return grid
```

**Step 2: Create Aligned Depth Map**
```python
def _create_depth_map(lidar_data, camera_frame) -> np.ndarray:
    """
    Project 3D LiDAR points → 2D camera pixel depth map for YOLO

    Process:
    1. Decode JPEG → numpy array (640x480)
    2. For each LiDAR point (x, y, z):
       - Project to camera pixel using pinhole model
       - pixel_x = width/2 + (x/z) * focal_length
       - pixel_y = height/2 + (y/z) * focal_length
    3. Accumulate depths at each pixel
    4. Use OpenCV inpainting to fill gaps

    Returns: 640x480 depth map (meters)
    """
    depth_map = np.zeros((480, 640), dtype=np.float32)
    point_count_map = np.zeros((480, 640), dtype=np.int32)

    for point in lidar_data['points']:
        x, y, z = point['x'], point['y'], point['z']

        # Pinhole camera projection
        pixel_x = int(width/2 + (x/z) * focal_length)
        pixel_y = int(height/2 + (y/z) * focal_length)

        if in_bounds(pixel_x, pixel_y):
            depth_map[pixel_y, pixel_x] += z
            point_count_map[pixel_y, pixel_x] += 1

    # Average overlapping points
    valid_mask = point_count_map > 0
    depth_map[valid_mask] /= point_count_map[valid_mask]

    # Fill gaps with OpenCV inpainting
    mask = (depth_map == 0).astype(np.uint8)
    depth_map = cv2.inpaint(depth_map, mask, 3, cv2.INPAINT_TELEA)

    return depth_map
```

**Step 3: YOLO Detection with Depth**
```python
# Detection Loop uses fused depth map
fused_data = state_manager.get_fused_data()
depth_map = fused_data['depth_map']  # From fusion loop

# YOLO detects objects
detections = yolo.detect(camera_frame)
# [{"label": "cup", "bbox": [100, 150, 80, 120], "confidence": 0.85}]

# Enrich with depth from fusion
for det in detections:
    bbox_center_x = det['bbox'][0] + det['bbox'][2] / 2
    bbox_center_y = det['bbox'][1] + det['bbox'][3] / 2

    # Look up depth at bounding box center
    depth = depth_map[int(bbox_center_y), int(bbox_center_x)]

    # Convert to 3D position
    det['depth'] = depth
    det['position'] = pixel_to_world(bbox_center_x, bbox_center_y, depth)
```

### Complete Sensor Fusion Data Flow

```
iPhone sends LiDAR point cloud (WebSocket)
ESP32-CAM sends RGB image (WebSocket)
    ↓
Fusion Loop (500ms interval)
    ↓
┌─────────────────────────────────────────┐
│  3D Point Cloud Processing              │
│  • Filter by height (5cm - 50cm)        │
│  • Robot frame → World frame            │
│  • Project to 2D grid (100x100)         │
│  • Discretize to 0.1m cells             │
│  • Count points per cell                │
│  • Mark obstacles (≥5 points)           │
└─────────────────────────────────────────┘
    ↓
Occupancy Grid (100x100, uint8)
    ↓ Sent to Path Planning Loop
D* Lite uses for pathfinding

    ↓
┌─────────────────────────────────────────┐
│  Camera-LiDAR Alignment                 │
│  • Decode JPEG camera frame             │
│  • Pinhole projection: 3D → 2D pixel    │
│  • Accumulate depths per pixel          │
│  • OpenCV inpainting for gaps           │
└─────────────────────────────────────────┘
    ↓
Aligned Depth Map (640x480, float32)
    ↓ Sent to Detection Loop
YOLO enriches detections with depth

    ↓
Detection Loop (3-5s interval)
    ↓
YOLO detects: "cup" at bbox [100, 150, 80, 120]
Depth lookup: depth_map[150, 140] = 2.3m
Position: (2.3, 1.8, 0.2) in world coordinates
    ↓
Gemini ADK Planning Loop
Receives: "cup detected at 2.3m with 85% confidence"
Decides: "Navigate to cup - likely target"
Sets goal: (2.3, 1.8)
    ↓
D* Lite Path Planning
Uses occupancy grid to find path avoiding obstacles
    ↓
Motor Control Loop
Drives robot to target
```

---

## System Integration Summary

### Concurrent Loop Architecture

Our system runs 5 concurrent async loops, each with specific responsibilities:

| Loop | Frequency | Purpose | Dependencies |
|------|-----------|---------|--------------|
| **Fusion Loop** | 2 Hz (500ms) | LiDAR+Camera fusion → Occupancy grid + Depth map | LiDAR, Camera |
| **Path Planning** | 1 Hz (1s) | D* Lite pathfinding with replanning | Occupancy grid, Odometry |
| **Planning Loop** | 1 Hz (1s) | Gemini ADK decision-making | Detections, Path status |
| **Detection Loop** | 0.2-0.3 Hz (3-5s) | YOLO object detection with depth | Camera, Depth map |
| **Motor Control** | 0.3 Hz (3s) | Drive to waypoints | Path waypoints, Odometry |

### Key Technologies

- **Google ADK**: Stateful AI agent with tool access for autonomous decision-making
- **D* Lite**: Incremental pathfinding with efficient replanning
- **Sensor Fusion**: OpenCV-based LiDAR-Camera alignment for 3D perception
- **YOLO11n**: Real-time object detection (80+ classes)
- **FastAPI + WebSockets**: Real-time sensor streaming and motor control

### Why This Architecture Works

1. **Human-out-of-the-Loop**: User gives mission, system operates autonomously
2. **Real-time Replanning**: D* Lite adapts to new obstacles incrementally
3. **Stateful AI**: Google ADK maintains context across decision cycles
4. **Sensor Fusion**: Combined LiDAR+Camera provides rich 3D perception
5. **Concurrent Processing**: All loops run independently at optimal frequencies

The result: A fully autonomous robot that perceives, reasons, plans, and acts without human intervention.
