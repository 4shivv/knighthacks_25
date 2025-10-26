"""
Shared state manager for coordinating data between all concurrent loops.
Thread-safe in-memory storage for real-time performance.
"""

from threading import Lock
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, field
import time
import numpy as np


@dataclass
class DetectedObject:
    """Represents a detected object with metadata."""
    label: str
    confidence: float
    bbox: List[float]  # [x, y, width, height]
    depth: Optional[float] = None  # Distance in meters
    position: Optional['Position'] = None  # 3D position in world coordinates
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

        # Pathfinding state (D* Lite integration)
        self.occupancy_grid: Optional[np.ndarray] = None  # 2D grid for D* Lite (0=free, 255=obstacle)
        self.occupancy_grid_timestamp: float = 0
        self.current_path: List[Tuple[int, int]] = []  # D* Lite path (grid coordinates)
        self.next_waypoint: Optional[Tuple[float, float]] = None  # Next waypoint in world coordinates
        self.goal_position: Optional[Tuple[int, int]] = None  # Goal in grid coordinates
        self.path_status: str = "NO_GOAL"  # NO_GOAL, PLANNING, PATH_FOUND, BLOCKED, EXPLORATION

        # Motor control (replaces text instructions for RC car)
        self.motor_command_queue: List[Dict[str, Any]] = []
        self.last_motor_command: Optional[Dict[str, Any]] = None
        self.last_motor_command_timestamp: float = 0

        # Odometry / Movement state
        self.current_heading: float = 0.0  # Heading in degrees (0 = North, 90 = East)
        self.wheel_encoder_left: int = 0
        self.wheel_encoder_right: int = 0
        self.velocity: float = 0.0  # Current velocity in m/s

        # Statistics
        self.total_motor_commands_sent: int = 0
        self.total_detections: int = 0
        self.dstar_replan_count: int = 0

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

    def get_fused_data(self) -> Optional[Dict[str, Any]]:
        """Get fused sensor data (depth map + occupancy grid)."""
        with self._lock:
            return self.fused_data

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

    # ==================== Motor Command Methods ====================

    def add_motor_command(self, command: Dict[str, Any]):
        """Add motor command to queue."""
        with self._lock:
            self.motor_command_queue.append(command)
            self.last_motor_command = command
            self.last_motor_command_timestamp = time.time()
            self.total_motor_commands_sent += 1

    def pop_motor_command(self) -> Optional[Dict[str, Any]]:
        """Get and remove next motor command from queue."""
        with self._lock:
            if self.motor_command_queue:
                return self.motor_command_queue.pop(0)
            return None

    def clear_motor_command_queue(self):
        """Clear all pending motor commands."""
        with self._lock:
            self.motor_command_queue.clear()

    # ==================== Pathfinding Methods ====================

    def update_occupancy_grid(self, grid: np.ndarray, timestamp: float = None):
        """Update occupancy grid from LiDAR data."""
        with self._lock:
            self.occupancy_grid = grid.copy()
            self.occupancy_grid_timestamp = timestamp or time.time()

    def update_path(self, path: List[Tuple[int, int]], status: str = "PATH_FOUND"):
        """Update current path from D* Lite."""
        with self._lock:
            self.current_path = path
            self.path_status = status
            if len(path) > 1:
                # Set next waypoint (second point in path, first is current position)
                self.next_waypoint = path[1]
            elif len(path) == 1:
                # Already at goal
                self.next_waypoint = path[0]
            else:
                self.next_waypoint = None

    def set_goal_position(self, goal: Tuple[int, int]):
        """Set goal position for pathfinding."""
        with self._lock:
            self.goal_position = goal
            self.path_status = "PLANNING"

    def update_next_waypoint(self, waypoint: Optional[Tuple[float, float]]):
        """
        Update the next waypoint from path planning.

        Args:
            waypoint: Next waypoint in world coordinates (x, y) or None if at goal
        """
        with self._lock:
            self.next_waypoint = waypoint

    def clear_goal(self):
        """Clear current goal and path."""
        with self._lock:
            self.goal_position = None
            self.current_path = []
            self.next_waypoint = None
            self.path_status = "NO_GOAL"

    def increment_replan_count(self):
        """Increment D* Lite replan counter."""
        with self._lock:
            self.dstar_replan_count += 1

    # ==================== Odometry Methods ====================

    def update_odometry(self, heading: float = None, velocity: float = None,
                       encoder_left: int = None, encoder_right: int = None):
        """Update odometry data from RC car."""
        with self._lock:
            if heading is not None:
                self.current_heading = heading % 360  # Normalize to 0-360
            if velocity is not None:
                self.velocity = velocity
            if encoder_left is not None:
                self.wheel_encoder_left = encoder_left
            if encoder_right is not None:
                self.wheel_encoder_right = encoder_right

    @property
    def target_object(self) -> Optional[DetectedObject]:
        """
        Get the target object based on current mission objective.

        Returns:
            The detected object matching the objective, or None
        """
        with self._lock:
            if not self.objective or not self.detected_objects:
                return None

            # Find objects matching objective
            objective_lower = self.objective.lower()
            for label, objects_list in self.detected_objects.items():
                if label.lower() in objective_lower or objective_lower in label.lower():
                    if objects_list:
                        # Return the most confident detection
                        return max(objects_list, key=lambda obj: obj.confidence)

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
            self.total_motor_commands_sent = 0
            self.total_detections = 0
            self.dstar_replan_count = 0
            # Reset pathfinding state
            self.clear_goal()
            # Reset data
            self.detected_objects = {}
            self.position_history = [self.current_position]
            # Reset motor commands
            self.motor_command_queue.clear()

    def complete_mission(self, found_object: Optional[DetectedObject] = None):
        """
        Mark mission as complete.

        Args:
            found_object: The detected object that completed the mission (if any)
        """
        with self._lock:
            self.mission_active = False
            self.mission_complete = True
            if found_object:
                # Log successful find
                import logging
                logger = logging.getLogger(__name__)
                logger.info(f"Mission complete! Found {found_object.label} at depth {found_object.depth:.2f}m with confidence {found_object.confidence:.2f}")

    def get_mission_stats(self) -> Dict[str, Any]:
        """Get mission statistics."""
        with self._lock:
            elapsed_time = time.time() - self.mission_start_time if self.mission_start_time > 0 else 0
            return {
                "objective": self.objective,
                "elapsed_time_seconds": elapsed_time,
                "motor_commands_sent": self.total_motor_commands_sent,
                "total_detections": self.total_detections,
                "positions_visited": len(self.position_history),
                "dstar_replans": self.dstar_replan_count,
                "mission_complete": self.mission_complete,
                "path_status": self.path_status
            }

    def get_state_snapshot(self) -> Dict[str, Any]:
        """Get a snapshot of current state for debugging."""
        with self._lock:
            return {
                "mission_active": self.mission_active,
                "objective": self.objective,
                "detected_objects_count": sum(len(objs) for objs in self.detected_objects.values()),
                "motor_command_queue_length": len(self.motor_command_queue),
                "last_motor_command": self.last_motor_command,
                "current_position": {
                    "x": self.current_position.x,
                    "y": self.current_position.y,
                    "z": self.current_position.z
                },
                "current_heading": self.current_heading,
                "velocity": self.velocity,
                "path_status": self.path_status,
                "next_waypoint": self.next_waypoint,
                "path_length": len(self.current_path),
                "goal_position": self.goal_position,
                "has_occupancy_grid": self.occupancy_grid is not None,
                "sensor_data_fresh": self.has_fresh_sensor_data()
            }


# Global state manager instance
state_manager = StateManager()
