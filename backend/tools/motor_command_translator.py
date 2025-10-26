"""
Motor Command Translator for ELEGOO Smart Robot Car V4.0
Translates high-level navigation commands to ELEGOO motor control format.

Based on ELEGOO Arduino code analysis:
- Motor A (Right): PIN_Motor_PWMA (speed 0-255), PIN_Motor_AIN_1 (direction)
  - AIN_1 = LOW → Forward
  - AIN_1 = HIGH → Backward
- Motor B (Left): PIN_Motor_PWMB (speed 0-255), PIN_Motor_BIN_1 (direction)
  - BIN_1 = HIGH → Forward (INVERTED!)
  - BIN_1 = LOW → Backward (INVERTED!)

JSON Command Format (N=4 - Direct motor speed control):
{
    "N": 4,
    "H": "command_id",
    "D1": left_motor_speed (0-255),
    "D2": right_motor_speed (0-255)
}
"""

import math
import logging
from typing import Dict, Any, Tuple, Optional

logger = logging.getLogger(__name__)


class MotorCommandTranslator:
    """
    Translates high-level navigation commands to ELEGOO motor protocol.
    """

    # ELEGOO motor constants
    MAX_SPEED = 255
    MIN_SPEED = 0

    # Calibration constants (adjust these based on testing)
    MAX_SPEED_M_S = 0.5  # Maximum speed in meters/second at PWM=255
    WHEEL_BASE = 0.15     # Distance between wheels in meters
    TURN_RATE = 1.0       # Turning rate multiplier

    def __init__(self):
        self.command_counter = 0
        logger.info("Motor Command Translator initialized")

    def translate(self, command: Dict[str, Any], current_position: Tuple[float, float] = (0, 0),
                  current_heading: float = 0.0) -> Dict[str, Any]:
        """
        Translate high-level command to ELEGOO JSON format.

        Args:
            command: High-level command dict with "command" key
            current_position: Current (x, y) position in meters
            current_heading: Current heading in radians

        Returns:
            ELEGOO JSON command dict: {"N": 4, "H": "xxx", "D1": left_speed, "D2": right_speed}
        """
        self.command_counter += 1
        cmd_id = f"{self.command_counter:03d}"

        command_type = command.get("command", "STOP")

        if command_type == "STOP" or command_type == "EMERGENCY_STOP":
            return self._create_stop_command(cmd_id)

        elif command_type == "MOVE_TO_WAYPOINT":
            return self._create_waypoint_command(
                command, current_position, current_heading, cmd_id
            )

        elif command_type == "ROTATE_SCAN":
            return self._create_rotate_command(command, cmd_id)

        elif command_type == "BACKUP":
            return self._create_backup_command(command, cmd_id)

        elif command_type == "FORWARD":
            return self._create_forward_command(command, cmd_id)

        else:
            logger.warning(f"Unknown command type: {command_type}, defaulting to STOP")
            return self._create_stop_command(cmd_id)

    def _create_stop_command(self, cmd_id: str) -> Dict[str, Any]:
        """Create STOP command."""
        return {
            "N": 4,
            "H": cmd_id,
            "D1": 0,  # Left motor
            "D2": 0   # Right motor
        }

    def _create_waypoint_command(
        self,
        command: Dict[str, Any],
        current_position: Tuple[float, float],
        current_heading: float,
        cmd_id: str
    ) -> Dict[str, Any]:
        """
        Create command to move toward a waypoint.
        Uses differential drive control.
        """
        waypoint = command.get("waypoint")
        if not waypoint or len(waypoint) < 2:
            logger.error("Invalid waypoint in command")
            return self._create_stop_command(cmd_id)

        target_x, target_y = waypoint[0], waypoint[1]
        current_x, current_y = current_position

        # Calculate vector to waypoint
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)

        # If very close to waypoint, stop
        if distance < 0.1:
            return self._create_stop_command(cmd_id)

        # Calculate required heading
        target_heading = math.atan2(dy, dx)

        # Calculate heading error (normalize to -π to π)
        heading_error = target_heading - current_heading
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Get base speed from command (0-100%)
        speed_percent = command.get("speed", 60)
        base_speed = int((speed_percent / 100.0) * self.MAX_SPEED)

        # Differential drive control
        # If heading error is large, prioritize turning
        if abs(heading_error) > math.radians(10):
            # Need significant turning
            turn_speed = min(abs(heading_error) / math.pi * base_speed, base_speed)

            if heading_error > 0:
                # Turn left: left motor slower
                left_speed = max(0, int(base_speed * 0.3))
                right_speed = int(base_speed * 0.8)
            else:
                # Turn right: right motor slower
                left_speed = int(base_speed * 0.8)
                right_speed = max(0, int(base_speed * 0.3))
        else:
            # Nearly aligned, go forward with minor correction
            correction = int(heading_error / math.radians(10) * base_speed * 0.3)
            left_speed = base_speed - correction
            right_speed = base_speed + correction

        # Clamp to valid range
        left_speed = max(self.MIN_SPEED, min(self.MAX_SPEED, left_speed))
        right_speed = max(self.MIN_SPEED, min(self.MAX_SPEED, right_speed))

        return {
            "N": 4,
            "H": cmd_id,
            "D1": int(left_speed),
            "D2": int(right_speed)
        }

    def _create_rotate_command(self, command: Dict[str, Any], cmd_id: str) -> Dict[str, Any]:
        """
        Create command to rotate in place.
        Positive angle = rotate left (counter-clockwise)
        Negative angle = rotate right (clockwise)
        """
        angle = command.get("angle", 45)
        speed_percent = command.get("speed", 30)
        base_speed = int((speed_percent / 100.0) * self.MAX_SPEED)

        # For rotation in place: motors turn in opposite directions
        # Left motor backward, right motor forward = rotate right
        # Left motor forward, right motor backward = rotate left

        if angle > 0:
            # Rotate left (counter-clockwise)
            left_speed = -base_speed  # Negative means backward (will be handled by direction flag)
            right_speed = base_speed
        else:
            # Rotate right (clockwise)
            left_speed = base_speed
            right_speed = -base_speed

        # Note: ELEGOO uses separate direction pins, so we send absolute speeds
        # The Arduino code will handle direction based on sign
        # For now, we'll use forward rotation method

        return {
            "N": 4,
            "H": cmd_id,
            "D1": base_speed if angle < 0 else base_speed,
            "D2": base_speed if angle > 0 else base_speed
        }

    def _create_backup_command(self, command: Dict[str, Any], cmd_id: str) -> Dict[str, Any]:
        """
        Create command to move backward.
        """
        speed_percent = command.get("speed", 40)
        base_speed = int((speed_percent / 100.0) * self.MAX_SPEED)

        # Note: For backward movement, we need to send negative speeds
        # or use a different command mode. For now, using mode N=3 might be better
        # But we'll stick with N=4 and let the motor control loop handle direction

        # For backward with N=4, we need to invert motor directions
        # This will require special handling in motor_control_loop

        return {
            "N": 3,  # Use direction control mode instead
            "H": cmd_id,
            "D1": 2,  # Direction: 2 = Backward
            "D2": base_speed
        }

    def _create_forward_command(self, command: Dict[str, Any], cmd_id: str) -> Dict[str, Any]:
        """Create command to move straight forward."""
        speed_percent = command.get("speed", 60)
        base_speed = int((speed_percent / 100.0) * self.MAX_SPEED)

        return {
            "N": 4,
            "H": cmd_id,
            "D1": base_speed,
            "D2": base_speed
        }

    def estimate_duration(self, command: Dict[str, Any], distance: float = None) -> float:
        """
        Estimate command execution duration in seconds.

        Args:
            command: The motor command
            distance: Optional distance to target (meters)

        Returns:
            Estimated duration in seconds
        """
        command_type = command.get("command", "STOP")

        if command_type == "STOP" or command_type == "EMERGENCY_STOP":
            return 0.1

        elif command_type == "ROTATE_SCAN":
            angle = abs(command.get("angle", 45))
            speed = command.get("speed", 30)
            # Rough estimate: 1 degree per 0.05 seconds at medium speed
            return (angle / speed) * 2.0

        elif command_type == "BACKUP":
            backup_distance = command.get("distance", 0.3)
            speed = command.get("speed", 40)
            # Estimate time based on speed
            return (backup_distance / (self.MAX_SPEED_M_S * speed / 100.0))

        elif command_type == "MOVE_TO_WAYPOINT" and distance:
            speed_percent = command.get("speed", 60)
            estimated_speed = self.MAX_SPEED_M_S * (speed_percent / 100.0)
            return distance / estimated_speed if estimated_speed > 0 else 3.0

        else:
            # Default: 3 seconds
            return 3.0


# Global instance
motor_translator = MotorCommandTranslator()
