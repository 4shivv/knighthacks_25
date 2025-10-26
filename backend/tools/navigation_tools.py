"""
Navigation function tools for the RC car autonomous navigation agent.
These tools provide structured navigation commands and pathfinding integration.
"""

from typing import Dict, Any, Tuple, List, Optional
import logging
import math

logger = logging.getLogger(__name__)


def calculate_distance_to_object(
    object_label: str,
    detected_objects: list
) -> Dict[str, Any]:
    """
    Calculate distance to a specific detected object.

    Args:
        object_label: The label of the object to find (e.g., 'cup', 'mug')
        detected_objects: List of currently detected objects with depth info

    Returns:
        Dictionary with distance information or error
    """
    try:
        # Find matching objects
        matches = [
            obj for obj in detected_objects
            if obj.get('label', '').lower() == object_label.lower()
        ]

        if not matches:
            return {
                "status": "not_found",
                "message": f"No {object_label} detected in current view"
            }

        # Find closest match
        closest = min(matches, key=lambda x: x.get('depth', float('inf')))

        return {
            "status": "success",
            "object": object_label,
            "distance_meters": closest.get('depth', 0),
            "confidence": closest.get('confidence', 0),
            "position": closest.get('center', [0, 0])
        }

    except Exception as e:
        logger.error(f"Error calculating distance: {e}")
        return {
            "status": "error",
            "message": str(e)
        }


def suggest_movement_direction(
    target_object: str,
    detected_objects: list,
    image_width: int = 640
) -> Dict[str, Any]:
    """
    Suggest which direction to move based on object position in frame.

    Args:
        target_object: Object to move towards
        detected_objects: List of detected objects
        image_width: Width of camera frame (default 640)

    Returns:
        Dictionary with movement suggestion
    """
    try:
        # Find target object
        matches = [
            obj for obj in detected_objects
            if obj.get('label', '').lower() == target_object.lower()
        ]

        if not matches:
            return {
                "status": "not_found",
                "suggestion": "Turn slowly in place to scan for object"
            }

        # Get closest match
        closest = min(matches, key=lambda x: x.get('depth', float('inf')))
        center_x = closest.get('center', [image_width // 2, 0])[0]

        # Determine direction based on position in frame
        frame_center = image_width / 2
        tolerance = image_width * 0.1  # 10% tolerance

        if center_x < frame_center - tolerance:
            direction = "left"
            angle = int((frame_center - center_x) / frame_center * 30)  # Max 30 degrees
        elif center_x > frame_center + tolerance:
            direction = "right"
            angle = int((center_x - frame_center) / frame_center * 30)
        else:
            direction = "straight"
            angle = 0

        distance = closest.get('depth', 0)

        return {
            "status": "success",
            "object": target_object,
            "direction": direction,
            "angle_degrees": angle,
            "distance_meters": distance,
            "suggestion": f"Move {direction}" + (f" {angle}°" if angle > 0 else "") + f" toward {target_object} ({distance:.1f}m away)"
        }

    except Exception as e:
        logger.error(f"Error suggesting movement: {e}")
        return {
            "status": "error",
            "message": str(e)
        }


def check_path_clear(
    detected_objects: list,
    min_safe_distance: float = 0.5
) -> Dict[str, Any]:
    """
    Check if the path ahead is clear of obstacles.

    Args:
        detected_objects: List of detected objects with depth
        min_safe_distance: Minimum safe distance in meters

    Returns:
        Dictionary with path clearance status
    """
    try:
        # Find objects in path (center of frame, close distance)
        obstacles_ahead = [
            obj for obj in detected_objects
            if obj.get('depth', float('inf')) < min_safe_distance
        ]

        if not obstacles_ahead:
            return {
                "status": "clear",
                "message": "Path is clear to move forward",
                "safe_distance": min_safe_distance
            }

        # Find closest obstacle
        closest_obstacle = min(obstacles_ahead, key=lambda x: x.get('depth', float('inf')))

        return {
            "status": "blocked",
            "message": f"Obstacle detected: {closest_obstacle.get('label', 'unknown')} at {closest_obstacle.get('depth', 0):.1f}m",
            "obstacle": closest_obstacle.get('label', 'unknown'),
            "distance": closest_obstacle.get('depth', 0),
            "safe_distance": min_safe_distance
        }

    except Exception as e:
        logger.error(f"Error checking path: {e}")
        return {
            "status": "error",
            "message": str(e)
        }


def describe_environment(detected_objects: list) -> Dict[str, Any]:
    """
    Provide a summary description of the detected environment.

    Args:
        detected_objects: List of all detected objects

    Returns:
        Dictionary with environment description
    """
    try:
        if not detected_objects:
            return {
                "status": "empty",
                "message": "No objects detected in current view"
            }

        # Group objects by label
        object_counts = {}
        for obj in detected_objects:
            label = obj.get('label', 'unknown')
            object_counts[label] = object_counts.get(label, 0) + 1

        # Find closest object
        closest = min(detected_objects, key=lambda x: x.get('depth', float('inf')))

        # Build description
        items = [f"{count} {label}{'s' if count > 1 else ''}" for label, count in object_counts.items()]
        description = "I can see: " + ", ".join(items)

        return {
            "status": "success",
            "total_objects": len(detected_objects),
            "object_types": len(object_counts),
            "objects": object_counts,
            "closest_object": closest.get('label', 'unknown'),
            "closest_distance": closest.get('depth', 0),
            "description": description
        }

    except Exception as e:
        logger.error(f"Error describing environment: {e}")
        return {
            "status": "error",
            "message": str(e)
        }


# ==================== NEW D* LITE INTEGRATION TOOLS ====================

def check_goal_reached(
    target_object: str,
    detected_objects: list,
    success_distance_threshold: float = 0.5,
    success_confidence_threshold: float = 0.95
) -> Dict[str, Any]:
    """
    Check if the RC car has reached the goal object.

    Args:
        target_object: The object we're searching for
        detected_objects: List of detected objects
        success_distance_threshold: Max distance to consider goal reached (meters)
        success_confidence_threshold: Min confidence to consider detection valid

    Returns:
        Dictionary indicating if goal is reached
    """
    try:
        # Find matching objects
        matches = [
            obj for obj in detected_objects
            if obj.get('label', '').lower() == target_object.lower()
        ]

        if not matches:
            return {
                "goal_reached": False,
                "reason": "Target not visible",
                "status": "searching"
            }

        # Find best match (highest confidence)
        best_match = max(matches, key=lambda x: x.get('confidence', 0))

        # Check if it meets success criteria
        distance = best_match.get('depth', float('inf'))
        confidence = best_match.get('confidence', 0)

        if confidence >= success_confidence_threshold and distance <= success_distance_threshold:
            return {
                "goal_reached": True,
                "status": "success",
                "distance": distance,
                "confidence": confidence,
                "object": target_object,
                "message": f"Goal reached! {target_object} found at {distance:.2f}m with {confidence:.2%} confidence"
            }

        # Close but not quite there
        if distance <= success_distance_threshold * 1.5:
            return {
                "goal_reached": False,
                "status": "approaching",
                "distance": distance,
                "confidence": confidence,
                "message": f"{target_object} detected nearby ({distance:.2f}m), moving closer..."
            }

        # Visible but far
        return {
            "goal_reached": False,
            "status": "visible",
            "distance": distance,
            "confidence": confidence,
            "message": f"{target_object} visible at {distance:.2f}m, navigating toward it"
        }

    except Exception as e:
        logger.error(f"Error checking goal: {e}")
        return {
            "goal_reached": False,
            "status": "error",
            "message": str(e)
        }


def validate_path_safety(
    current_path: List[Tuple[int, int]],
    detected_objects: list,
    occupancy_grid_available: bool,
    min_clearance: float = 0.3
) -> Dict[str, Any]:
    """
    Validate if the current D* Lite path is still safe to follow.

    Args:
        current_path: D* Lite path as list of grid coordinates
        detected_objects: Currently detected obstacles
        occupancy_grid_available: Whether occupancy grid exists
        min_clearance: Minimum safe distance from obstacles (meters)

    Returns:
        Dictionary with path safety status
    """
    try:
        if not current_path:
            return {
                "safe": False,
                "reason": "No path available",
                "action": "REQUEST_REPLAN"
            }

        if not occupancy_grid_available:
            return {
                "safe": False,
                "reason": "No occupancy grid available",
                "action": "WAIT_FOR_SENSORS"
            }

        # Check for unexpected obstacles in path
        close_obstacles = [
            obj for obj in detected_objects
            if obj.get('depth', float('inf')) < min_clearance
        ]

        if close_obstacles:
            closest = min(close_obstacles, key=lambda x: x.get('depth', float('inf')))
            return {
                "safe": False,
                "reason": f"Obstacle too close: {closest.get('label', 'unknown')} at {closest.get('depth', 0):.2f}m",
                "obstacle": closest.get('label'),
                "distance": closest.get('depth', 0),
                "action": "EMERGENCY_STOP"
            }

        # Path appears safe
        return {
            "safe": True,
            "path_length": len(current_path),
            "message": "Path is clear and safe to follow"
        }

    except Exception as e:
        logger.error(f"Error validating path: {e}")
        return {
            "safe": False,
            "reason": str(e),
            "action": "EMERGENCY_STOP"
        }


def calculate_motor_command_for_waypoint(
    current_position: Tuple[float, float],
    target_waypoint: Tuple[float, float],
    current_heading: float,
    base_speed: int = 60
) -> Dict[str, Any]:
    """
    Calculate motor command (PWM values) to reach a waypoint.

    Args:
        current_position: Current (x, y) position
        target_waypoint: Target (x, y) waypoint
        current_heading: Current heading in degrees (0=North, 90=East)
        base_speed: Base motor speed (0-100)

    Returns:
        Dictionary with motor command parameters
    """
    try:
        # Calculate vector to waypoint
        dx = target_waypoint[0] - current_position[0]
        dy = target_waypoint[1] - current_position[1]

        # Calculate distance
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate required heading (in degrees)
        target_heading = math.degrees(math.atan2(dy, dx))

        # Normalize to 0-360
        target_heading = (target_heading + 360) % 360

        # Calculate heading error
        heading_error = (target_heading - current_heading + 180) % 360 - 180

        # Adjust motor speeds based on heading error (differential drive)
        if abs(heading_error) < 5:  # Nearly aligned, go straight
            left_motor = base_speed
            right_motor = base_speed
            command_type = "FORWARD"
        elif heading_error > 0:  # Need to turn right
            turn_factor = min(abs(heading_error) / 30.0, 1.0)  # Max differential at 30°
            left_motor = base_speed
            right_motor = int(base_speed * (1 - turn_factor * 0.5))
            command_type = "TURN_RIGHT"
        else:  # Need to turn left
            turn_factor = min(abs(heading_error) / 30.0, 1.0)
            left_motor = int(base_speed * (1 - turn_factor * 0.5))
            right_motor = base_speed
            command_type = "TURN_LEFT"

        return {
            "command": "MOVE_TO_WAYPOINT",
            "waypoint": list(target_waypoint),
            "distance_to_waypoint": distance,
            "heading_error": heading_error,
            "motor_left": left_motor,
            "motor_right": right_motor,
            "speed": base_speed,
            "command_type": command_type,
            "duration_ms": int(distance * 1000)  # Rough estimate: 1 second per meter
        }

    except Exception as e:
        logger.error(f"Error calculating motor command: {e}")
        return {
            "command": "ERROR",
            "message": str(e)
        }


def get_exploration_command(
    last_motor_command: Optional[Dict[str, Any]] = None,
    scan_angle: int = 45
) -> Dict[str, Any]:
    """
    Generate exploration command when no goal is set.
    Uses systematic rotation to scan for target object.

    Args:
        last_motor_command: Previous motor command (to track exploration progress)
        scan_angle: Rotation angle for each scan step (degrees)

    Returns:
        Motor command for exploration
    """
    try:
        # Simple exploration: rotate in place to scan
        return {
            "command": "ROTATE_SCAN",
            "angle": scan_angle,
            "speed": 30,
            "direction": "clockwise",
            "message": f"Exploring: rotating {scan_angle}° to scan for target"
        }

    except Exception as e:
        logger.error(f"Error generating exploration command: {e}")
        return {
            "command": "ERROR",
            "message": str(e)
        }
