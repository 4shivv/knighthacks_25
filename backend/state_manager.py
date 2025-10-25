"""
Shared state manager for coordinating data between all concurrent loops.
Thread-safe in-memory storage for real-time performance.
"""

from threading import Lock
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field
import time


@dataclass
class DetectedObject:
    """Represents a detected object with metadata."""
    label: str
    confidence: float
    bbox: List[float]  # [x, y, width, height]
    depth: Optional[float] = None  # Distance in meters
    timestamp: float = field(default_factory=time.time)


@dataclass
class Position:
    """Represents a 3D position."""
    x: float
    y: float
    z: float
    timestamp: float = field(default_factory=time.time)


class StateManager:
    """
    Thread-safe shared state manager for all loops.
    Provides fast in-memory storage with locking for concurrent access.
    """

    def __init__(self):
        self._lock = Lock()

        # Mission state
        self.objective: str = ""
        self.mission_start_time: float = 0
        self.mission_active: bool = False
        self.mission_complete: bool = False

        # Sensor data
        self.last_lidar_data: Optional[Dict[str, Any]] = None
        self.last_camera_frame: Optional[Any] = None
        self.last_lidar_timestamp: float = 0
        self.last_camera_timestamp: float = 0

        # Fused data
        self.fused_data: Optional[Dict[str, Any]] = None
        self.fused_timestamp: float = 0

        # Detected objects
        self.detected_objects: Dict[str, List[DetectedObject]] = {}  # label -> list of objects
        self.last_detection_timestamp: float = 0

        # Position tracking
        self.current_position: Position = Position(0, 0, 0)
        self.position_history: List[Position] = []

        # Instruction queue
        self.instruction_queue: List[str] = []
        self.last_instruction: Optional[str] = None
        self.last_instruction_timestamp: float = 0

        # Statistics
        self.total_instructions_given: int = 0
        self.total_detections: int = 0

    def update_lidar(self, data: Dict[str, Any], timestamp: float):
        """Update LiDAR data."""
        with self._lock:
            self.last_lidar_data = data
            self.last_lidar_timestamp = timestamp

    def update_camera(self, frame: Any, timestamp: float):
        """Update camera frame."""
        with self._lock:
            self.last_camera_frame = frame
            self.last_camera_timestamp = timestamp

    def update_fused_data(self, data: Dict[str, Any], timestamp: float):
        """Update fused sensor data."""
        with self._lock:
            self.fused_data = data
            self.fused_timestamp = timestamp

    def add_detected_object(self, obj: DetectedObject):
        """Add a detected object."""
        with self._lock:
            if obj.label not in self.detected_objects:
                self.detected_objects[obj.label] = []
            self.detected_objects[obj.label].append(obj)
            self.total_detections += 1
            self.last_detection_timestamp = time.time()

    def get_detected_objects(self, label: Optional[str] = None) -> List[DetectedObject]:
        """Get detected objects, optionally filtered by label."""
        with self._lock:
            if label:
                return self.detected_objects.get(label, [])
            # Return all objects
            all_objects = []
            for objects in self.detected_objects.values():
                all_objects.extend(objects)
            return all_objects

    def clear_old_detections(self, max_age_seconds: float = 10.0):
        """Remove detections older than max_age_seconds."""
        with self._lock:
            current_time = time.time()
            for label in list(self.detected_objects.keys()):
                self.detected_objects[label] = [
                    obj for obj in self.detected_objects[label]
                    if current_time - obj.timestamp < max_age_seconds
                ]
                # Remove empty labels
                if not self.detected_objects[label]:
                    del self.detected_objects[label]

    def update_position(self, position: Position):
        """Update current position and add to history."""
        with self._lock:
            self.current_position = position
            self.position_history.append(position)
            # Keep only last 100 positions
            if len(self.position_history) > 100:
                self.position_history.pop(0)

    def add_instruction(self, instruction: str):
        """Add instruction to queue."""
        with self._lock:
            self.instruction_queue.append(instruction)
            self.last_instruction = instruction
            self.last_instruction_timestamp = time.time()
            self.total_instructions_given += 1

    def pop_instruction(self) -> Optional[str]:
        """Get and remove next instruction from queue."""
        with self._lock:
            if self.instruction_queue:
                return self.instruction_queue.pop(0)
            return None

    def has_fresh_sensor_data(self, timeout_ms: int = 2000) -> bool:
        """Check if both sensors have fresh data within timeout."""
        with self._lock:
            current_time = time.time()
            timeout_seconds = timeout_ms / 1000.0

            lidar_fresh = (current_time - self.last_lidar_timestamp) < timeout_seconds
            camera_fresh = (current_time - self.last_camera_timestamp) < timeout_seconds

            return lidar_fresh and camera_fresh

    def start_mission(self, objective: str):
        """Start a new mission."""
        with self._lock:
            self.objective = objective
            self.mission_start_time = time.time()
            self.mission_active = True
            self.mission_complete = False
            # Reset counters
            self.total_instructions_given = 0
            self.total_detections = 0
            self.detected_objects = {}
            self.position_history = [self.current_position]

    def complete_mission(self):
        """Mark mission as complete."""
        with self._lock:
            self.mission_active = False
            self.mission_complete = True

    def get_mission_stats(self) -> Dict[str, Any]:
        """Get mission statistics."""
        with self._lock:
            elapsed_time = time.time() - self.mission_start_time if self.mission_start_time > 0 else 0
            return {
                "objective": self.objective,
                "elapsed_time_seconds": elapsed_time,
                "instructions_given": self.total_instructions_given,
                "total_detections": self.total_detections,
                "positions_visited": len(self.position_history),
                "mission_complete": self.mission_complete
            }

    def get_state_snapshot(self) -> Dict[str, Any]:
        """Get a snapshot of current state for debugging."""
        with self._lock:
            return {
                "mission_active": self.mission_active,
                "objective": self.objective,
                "detected_objects_count": sum(len(objs) for objs in self.detected_objects.values()),
                "instruction_queue_length": len(self.instruction_queue),
                "last_instruction": self.last_instruction,
                "current_position": {
                    "x": self.current_position.x,
                    "y": self.current_position.y,
                    "z": self.current_position.z
                },
                "sensor_data_fresh": self.has_fresh_sensor_data()
            }


# Global state manager instance
state_manager = StateManager()
