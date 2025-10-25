"""
Navigation function tools for the blindfolded navigation agent.
These tools provide structured navigation commands.
"""

from typing import Dict, Any
import logging

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
