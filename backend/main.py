"""
Main FastAPI application entry point.
Initializes all components and starts the server with concurrent loops.
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import asyncio
import logging
from contextlib import asynccontextmanager

from config import settings
from state_manager import state_manager
from models.yolo_model import YOLOModel

# ADK imports
from agents.navigation_agent import navigation_agent, prepare_agent_state
from adk_session_manager import ADKSessionManager

# Configure logging
logging.basicConfig(
    level=logging.INFO if settings.debug else logging.WARNING,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Global instances (will be initialized on startup)
yolo_model: YOLOModel = None
adk_session_manager: ADKSessionManager = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup and shutdown events.
    """
    # STARTUP
    logger.info("=" * 60)
    logger.info("SERVER INITIALIZATION STARTED")
    logger.info("=" * 60)

    global yolo_model, adk_session_manager

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

    # 5. Start background loops (will be implemented in next phase)
    logger.info("Step 5: Starting concurrent loops...")
    logger.info("   (Loops will be implemented in next phase)")
    # TODO: Start sensor loop, detection loop, planning loop, etc.

    logger.info("=" * 60)
    logger.info("SERVER READY - Waiting for sensor input...")
    logger.info("=" * 60)
    logger.info(f"LiDAR WebSocket: ws://localhost:{settings.lidar_ws_port}")
    logger.info(f"Camera WebSocket: ws://localhost:{settings.camera_ws_port}")
    logger.info("=" * 60)

    yield

    # SHUTDOWN
    logger.info("Shutting down server...")
    # TODO: Clean up resources, stop loops


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
        "mission": state_manager.get_mission_stats(),
        "state": state_manager.get_state_snapshot()
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
    """
    await websocket.accept()
    logger.info("LiDAR client connected")

    try:
        while True:
            # Receive LiDAR data
            data = await websocket.receive_json()

            # Update state with timestamp
            import time
            timestamp = time.time()
            state_manager.update_lidar(data, timestamp)

            # Send acknowledgment
            await websocket.send_json({
                "status": "received",
                "timestamp": timestamp
            })

            logger.debug(f"LiDAR data received: {len(data.get('points', []))} points")

    except WebSocketDisconnect:
        logger.info("LiDAR client disconnected")
    except Exception as e:
        logger.error(f"LiDAR WebSocket error: {e}")


@app.websocket("/ws/camera")
async def camera_websocket(websocket: WebSocket):
    """
    WebSocket endpoint for ESP32 camera stream.
    Receives 2MP RGB image frames.
    """
    await websocket.accept()
    logger.info("Camera client connected")

    try:
        while True:
            # Receive camera frame (as base64 or binary)
            data = await websocket.receive_bytes()

            # Update state with timestamp
            import time
            timestamp = time.time()
            state_manager.update_camera(data, timestamp)

            # Send acknowledgment
            await websocket.send_json({
                "status": "received",
                "timestamp": timestamp
            })

            logger.debug(f"Camera frame received: {len(data)} bytes")

    except WebSocketDisconnect:
        logger.info("Camera client disconnected")
    except Exception as e:
        logger.error(f"Camera WebSocket error: {e}")


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
