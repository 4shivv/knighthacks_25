"""
Main FastAPI application entry point.
Initializes all components and starts the server with concurrent loops.
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import asyncio
import logging
import os
from contextlib import asynccontextmanager

from config import settings

# Set Google API key for ADK/GenAI SDK
os.environ['GOOGLE_API_KEY'] = settings.gemini_api_key
from state_manager import state_manager
from models.yolo_model import YOLOModel

# ADK imports
from agents.navigation_agent import navigation_agent, prepare_agent_state
from adk_session_manager import ADKSessionManager

# Loop imports
from loops.detection_loop import DetectionLoop
from loops.planning_loop import PlanningLoop
from loops.motor_control_loop import MotorControlLoop
from loops.path_planning_loop import PathPlanningLoop

# Tool imports
from tools.odometry import odometry

# Configure logging
logging.basicConfig(
    level=logging.INFO if settings.debug else logging.WARNING,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Global instances (will be initialized on startup)
yolo_model: YOLOModel = None
adk_session_manager: ADKSessionManager = None
detection_loop: DetectionLoop = None
planning_loop: PlanningLoop = None
motor_control_loop: MotorControlLoop = None
path_planning_loop: PathPlanningLoop = None
detection_task: asyncio.Task = None
planning_task: asyncio.Task = None
motor_control_task: asyncio.Task = None
path_planning_task: asyncio.Task = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup and shutdown events.
    """
    # STARTUP
    logger.info("=" * 60)
    logger.info("SERVER INITIALIZATION STARTED")
    logger.info("=" * 60)

    global yolo_model, adk_session_manager, detection_loop, planning_loop, motor_control_loop, path_planning_loop, detection_task, planning_task, motor_control_task, path_planning_task

    # 1. Load YOLO model
    logger.info("Step 1: Loading YOLO model...")
    try:
        yolo_model = YOLOModel(
            model_path=settings.yolo_model_path,
            confidence_threshold=settings.confidence_threshold
        )
        logger.info("✓ YOLO model loaded successfully")
    except Exception as e:
        logger.error(f"✗ Failed to load YOLO model: {e}")
        raise

    # 2. Initialize ADK Navigation Agent
    logger.info("Step 2: Initializing ADK Navigation Agent...")
    try:
        adk_session_manager = ADKSessionManager(
            agent=navigation_agent,
            app_name="blindfolded_navigation"
        )
        logger.info("✓ ADK Navigation Agent initialized successfully")
    except Exception as e:
        logger.error(f"✗ Failed to initialize ADK agent: {e}")
        raise

    # 3. Initialize shared state and start ADK session
    logger.info("Step 3: Initializing shared state manager...")
    state_manager.start_mission(settings.default_objective)
    logger.info(f"✓ Mission started: {settings.default_objective}")

    # 4. Start ADK session with initial state
    logger.info("Step 4: Starting ADK session...")
    try:
        initial_state = prepare_agent_state(state_manager)
        await adk_session_manager.start_new_session(initial_state)
        logger.info("✓ ADK session started successfully")
    except Exception as e:
        logger.error(f"✗ Failed to start ADK session: {e}")
        raise

    # 5. Start background loops
    logger.info("Step 5: Starting concurrent loops...")

    # Create detection loop
    detection_loop = DetectionLoop(
        state_manager=state_manager,
        yolo_model=yolo_model,
        interval_ms=settings.detection_loop_interval
    )
    # Start detection loop as async task
    detection_task = asyncio.create_task(detection_loop.run())
    logger.info(f"✓ Detection loop started ({settings.detection_loop_interval}ms interval)")

    # Create planning loop
    planning_loop = PlanningLoop(
        state_manager=state_manager,
        adk_session_manager=adk_session_manager,
        interval_ms=settings.planning_loop_interval
    )
    # Start planning loop as async task
    planning_task = asyncio.create_task(planning_loop.run())
    logger.info(f"✓ Planning loop started ({settings.planning_loop_interval}ms interval)")

    # Create motor control loop
    motor_control_loop = MotorControlLoop(
        state_manager=state_manager,
        odometry=odometry,
        interval_ms=3000  # 0.3 Hz (every 3 seconds)
    )
    # Start motor control loop as async task
    motor_control_task = asyncio.create_task(motor_control_loop.run())
    logger.info("✓ Motor control loop started (3000ms interval)")

    # Create path planning loop (D* Lite)
    path_planning_loop = PathPlanningLoop(
        state_manager=state_manager,
        odometry=odometry,
        grid_size_x=100,  # 10m x 0.1m resolution
        grid_size_y=100,
        grid_resolution=0.1,  # meters per cell
        interval_ms=1000  # 1 Hz
    )
    # Start path planning loop as async task
    path_planning_task = asyncio.create_task(path_planning_loop.run())
    logger.info("✓ Path planning loop started (1000ms interval)")

    logger.info("=" * 60)
    logger.info("SERVER READY - Waiting for sensor input...")
    logger.info("=" * 60)
    logger.info(f"LiDAR WebSocket: ws://localhost:{settings.server_port}/ws/lidar")
    logger.info(f"Camera WebSocket: ws://localhost:{settings.server_port}/ws/camera")
    logger.info(f"Arduino WebSocket: ws://localhost:{settings.server_port}/ws/arduino")
    logger.info("=" * 60)

    yield

    # SHUTDOWN
    logger.info("Shutting down server...")

    # Stop loops gracefully
    if detection_loop:
        logger.info("Stopping detection loop...")
        detection_loop.running = False
        if detection_task and not detection_task.done():
            await detection_task

    if planning_loop:
        logger.info("Stopping planning loop...")
        planning_loop.running = False
        if planning_task and not planning_task.done():
            await planning_task

    if motor_control_loop:
        logger.info("Stopping motor control loop...")
        motor_control_loop.running = False
        if motor_control_task and not motor_control_task.done():
            await motor_control_task

    if path_planning_loop:
        logger.info("Stopping path planning loop...")
        path_planning_loop.running = False
        if path_planning_task and not path_planning_task.done():
            await path_planning_task

    # End ADK session
    if adk_session_manager:
        logger.info("Ending ADK session...")
        await adk_session_manager.end_session()

    logger.info("Server shutdown complete")


# Create FastAPI app
app = FastAPI(
    title="Blindfolded Navigation Server",
    description="AI-powered navigation system using LiDAR, camera, YOLO, and Gemini",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ============================================================================
# REST API ENDPOINTS
# ============================================================================

@app.get("/")
async def root():
    """Root endpoint - server status."""
    return {
        "status": "running",
        "message": "Blindfolded Navigation Server",
        "version": "1.0.0"
    }


@app.get("/api/status")
async def get_status():
    """Get current system status."""
    return {
        "server": "running",
        "yolo_loaded": yolo_model is not None,
        "adk_agent_initialized": adk_session_manager is not None,
        "loops": {
            "detection": detection_loop.get_stats() if detection_loop else {"running": False},
            "planning": {"running": planning_loop.running} if planning_loop else {"running": False},
            "motor_control": motor_control_loop.get_stats() if motor_control_loop else {"running": False},
            "path_planning": path_planning_loop.get_stats() if path_planning_loop else {"running": False}
        },
        "mission": state_manager.get_mission_stats(),
        "state": state_manager.get_state_snapshot(),
        "odometry": {
            "position": odometry.get_position_tuple(),
            "heading_degrees": odometry.get_heading_degrees(),
            "velocity": odometry.get_velocity()
        }
    }


@app.get("/api/mission/stats")
async def get_mission_stats():
    """Get mission statistics."""
    return state_manager.get_mission_stats()


@app.post("/api/mission/start")
async def start_mission(objective: str = None):
    """Start a new mission."""
    obj = objective or settings.default_objective
    state_manager.start_mission(obj)
    logger.info(f"New mission started: {obj}")
    return {"status": "started", "objective": obj}


@app.post("/api/mission/complete")
async def complete_mission():
    """Mark mission as complete."""
    state_manager.complete_mission()
    stats = state_manager.get_mission_stats()
    logger.info(f"Mission completed! Stats: {stats}")
    return {"status": "completed", "stats": stats}


@app.get("/api/detections")
async def get_detections():
    """Get all detected objects."""
    objects = state_manager.get_detected_objects()
    return {
        "count": len(objects),
        "objects": [
            {
                "label": obj.label,
                "confidence": obj.confidence,
                "bbox": obj.bbox,
                "depth": obj.depth,
                "timestamp": obj.timestamp
            }
            for obj in objects
        ]
    }


@app.get("/api/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "timestamp": asyncio.get_event_loop().time()
    }


# ============================================================================
# WEBSOCKET ENDPOINTS
# ============================================================================

@app.websocket("/ws/lidar")
async def lidar_websocket(websocket: WebSocket):
    """
    WebSocket endpoint for iPhone LiDAR data stream.
    Receives 360° depth map + point cloud data.

    Expected data format:
    {
        "timestamp": float,  # Optional - server will use current time if missing
        "points": [
            {"x": float, "y": float, "z": float},
            ...
        ]
    }
    """
    await websocket.accept()
    logger.info("LiDAR client connected")

    try:
        while True:
            # Receive LiDAR data
            data = await websocket.receive_json()

            # Validate data format
            if not isinstance(data, dict):
                logger.warning("LiDAR data is not a dictionary, skipping")
                await websocket.send_json({
                    "status": "error",
                    "message": "Invalid data format: expected dictionary"
                })
                continue

            # Validate points field
            if "points" not in data:
                logger.warning("LiDAR data missing 'points' field, skipping")
                await websocket.send_json({
                    "status": "error",
                    "message": "Missing 'points' field"
                })
                continue

            points = data.get("points", [])
            if not isinstance(points, list):
                logger.warning("LiDAR 'points' field is not a list, skipping")
                await websocket.send_json({
                    "status": "error",
                    "message": "Invalid 'points' format: expected list"
                })
                continue

            # Validate point structure (at least first point if available)
            if len(points) > 0:
                first_point = points[0]
                if not isinstance(first_point, dict):
                    logger.warning("LiDAR points are not dictionaries, skipping")
                    await websocket.send_json({
                        "status": "error",
                        "message": "Invalid point format: expected dict with x, y, z"
                    })
                    continue

                # Check for required fields
                if not all(k in first_point for k in ['x', 'y', 'z']):
                    logger.warning("LiDAR point missing x, y, or z field, skipping")
                    await websocket.send_json({
                        "status": "error",
                        "message": "Invalid point format: missing x, y, or z field"
                    })
                    continue

            # Update state with timestamp
            import time
            timestamp = data.get("timestamp", time.time())
            state_manager.update_lidar(data, timestamp)

            # Send acknowledgment
            await websocket.send_json({
                "status": "received",
                "timestamp": timestamp,
                "points_count": len(points)
            })

            logger.debug(f"LiDAR data received: {len(points)} points")

    except WebSocketDisconnect:
        logger.info("LiDAR client disconnected")
    except Exception as e:
        logger.error(f"LiDAR WebSocket error: {e}")


@app.websocket("/ws/camera")
async def camera_websocket(websocket: WebSocket):
    """
    WebSocket endpoint for ESP32 camera stream.
    Receives 2MP RGB image frames.

    Expected data format: Raw JPEG bytes (compressed image)
    - Minimum size: 100 bytes (sanity check for valid JPEG)
    - Maximum size: 10 MB (reasonable for 2MP image)
    - Format: JPEG compressed image data
    """
    await websocket.accept()
    logger.info("Camera client connected")

    try:
        while True:
            # Receive camera frame (as binary JPEG data)
            data = await websocket.receive_bytes()

            # Validate data size
            MIN_FRAME_SIZE = 100  # bytes - minimum valid JPEG
            MAX_FRAME_SIZE = 10_000_000  # bytes - 10 MB max

            if len(data) < MIN_FRAME_SIZE:
                logger.warning(f"Camera frame too small ({len(data)} bytes), likely corrupted - skipping")
                await websocket.send_json({
                    "status": "error",
                    "message": f"Frame too small: {len(data)} bytes (min: {MIN_FRAME_SIZE})"
                })
                continue

            if len(data) > MAX_FRAME_SIZE:
                logger.warning(f"Camera frame too large ({len(data)} bytes), likely corrupted - skipping")
                await websocket.send_json({
                    "status": "error",
                    "message": f"Frame too large: {len(data)} bytes (max: {MAX_FRAME_SIZE})"
                })
                continue

            # Optional: Validate JPEG header (first 2 bytes should be 0xFF 0xD8)
            if len(data) >= 2:
                if data[0] != 0xFF or data[1] != 0xD8:
                    logger.warning("Camera frame does not start with JPEG header (0xFF 0xD8), may be corrupted")
                    await websocket.send_json({
                        "status": "warning",
                        "message": "Frame may be corrupted (invalid JPEG header)"
                    })
                    # Still accept it, might be a different format

            # Update state with timestamp
            import time
            timestamp = time.time()
            state_manager.update_camera(data, timestamp)

            # Send acknowledgment
            await websocket.send_json({
                "status": "received",
                "timestamp": timestamp,
                "frame_size_bytes": len(data)
            })

            logger.debug(f"Camera frame received: {len(data)} bytes")

    except WebSocketDisconnect:
        logger.info("Camera client disconnected")
    except Exception as e:
        logger.error(f"Camera WebSocket error: {e}")


@app.websocket("/ws/arduino")
async def arduino_websocket(websocket: WebSocket):
    """
    WebSocket endpoint for Arduino WiFi bridge.
    Receives motor commands and sends acknowledgments.
    Receives IMU/sensor data from Arduino.

    Expected command format (sent TO Arduino):
    {
        "N": 4,              # Command type (4 = motor speed control)
        "H": "command_id",   # Command ID for acknowledgment
        "D1": int,           # Left motor speed (0-255)
        "D2": int            # Right motor speed (0-255)
    }

    Expected acknowledgment format (received FROM Arduino):
    {
        "command_id": "xxx",
        "status": "ok" | "error",
        "message": "optional error message"
    }

    Expected sensor data format (received FROM Arduino):
    {
        "type": "imu",
        "yaw": float,        # IMU yaw angle in degrees
        "pitch": float,      # Optional
        "roll": float        # Optional
    }
    """
    await websocket.accept()
    logger.info("Arduino WiFi bridge connected")

    # Register this WebSocket with motor control loop
    if motor_control_loop:
        motor_control_loop.set_arduino_websocket(websocket)
        logger.info("Arduino WebSocket registered with motor control loop")

    try:
        while True:
            # Receive data from Arduino (acknowledgments or sensor data)
            data = await websocket.receive_json()

            # Check if this is an acknowledgment
            if "command_id" in data and "status" in data:
                # Store acknowledgment in state manager
                state_manager.last_motor_ack = data
                logger.debug(f"Arduino ACK: {data['command_id']} - {data['status']}")

            # Check if this is IMU data
            elif data.get("type") == "imu":
                yaw = data.get("yaw")
                if yaw is not None:
                    # Update odometry with IMU heading
                    odometry.update_from_imu(yaw)
                    # Update state manager with current heading
                    state_manager.update_odometry(
                        heading=yaw,
                        velocity=odometry.get_velocity()
                    )
                    logger.debug(f"Arduino IMU: yaw={yaw:.1f}°")

            else:
                logger.warning(f"Unknown Arduino message format: {data}")

    except WebSocketDisconnect:
        logger.warning("Arduino WiFi bridge disconnected")
        # Clear the WebSocket reference
        if motor_control_loop:
            motor_control_loop.arduino_ws = None
    except Exception as e:
        logger.error(f"Arduino WebSocket error: {e}")


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    logger.info("Starting Blindfolded Navigation Server...")

    uvicorn.run(
        "main:app",
        host=settings.server_host,
        port=settings.server_port,
        reload=settings.debug,
        log_level="info" if settings.debug else "warning"
    )
