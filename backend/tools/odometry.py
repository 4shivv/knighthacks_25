"""
Odometry module for ELEGOO Smart Robot Car V4.0
Tracks position using dead reckoning with optional IMU fusion.

Based on ELEGOO hardware:
- MPU6050 6-axis IMU (gyro + accelerometer) - provides yaw angle
- Wheel encoders (if available)
- Motor PWM commands (for velocity estimation)
"""

import math
import time
import logging
from typing import Tuple, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class OdometryState:
    """Current odometry state."""
    x: float = 0.0          # Position X in meters
    y: float = 0.0          # Position Y in meters
    heading: float = 0.0    # Heading in radians (0 = East, π/2 = North)
    velocity: float = 0.0   # Linear velocity in m/s
    last_update: float = 0.0


class Odometry:
    """
    Dead reckoning odometry with IMU fusion for ELEGOO Smart Robot Car.

    Integrates:
    1. Motor PWM commands → velocity estimation
    2. MPU6050 IMU yaw → accurate heading
    3. Optional: Wheel encoders (if available)
    """

    # Physical constants (calibrate these with real hardware)
    MAX_SPEED_M_S = 0.5      # Maximum speed at PWM=255 (m/s)
    WHEEL_BASE = 0.15        # Distance between wheels (meters)
    WHEEL_DIAMETER = 0.065   # Wheel diameter (meters)

    def __init__(self):
        self.state = OdometryState()
        self.state.last_update = time.time()

        # Calibration factors (adjust based on testing)
        self.speed_calibration = 1.0  # Multiply estimated speed by this
        self.turn_calibration = 1.0   # Multiply turning rate by this

        logger.info("Odometry initialized at origin (0, 0)")

    def update_from_motor_commands(
        self,
        left_speed_pwm: int,
        right_speed_pwm: int,
        imu_yaw_degrees: Optional[float] = None
    ):
        """
        Update position based on motor PWM commands.

        Args:
            left_speed_pwm: Left motor PWM (0-255)
            right_speed_pwm: Right motor PWM (0-255)
            imu_yaw_degrees: Optional IMU yaw angle in degrees (0-360)
        """
        current_time = time.time()
        dt = current_time - self.state.last_update

        if dt <= 0:
            return  # No time elapsed

        # Convert PWM to velocity (m/s)
        v_left = (left_speed_pwm / 255.0) * self.MAX_SPEED_M_S * self.speed_calibration
        v_right = (right_speed_pwm / 255.0) * self.MAX_SPEED_M_S * self.speed_calibration

        # Calculate linear and angular velocity
        v_linear = (v_left + v_right) / 2.0

        # Update heading
        if imu_yaw_degrees is not None:
            # Use IMU heading (more accurate!)
            # Convert from degrees to radians
            # IMU convention: 0° = North, 90° = East
            # Our convention: 0 = East, π/2 = North
            # So we need to convert: our_heading = (90 - imu_yaw) in degrees, then to radians
            self.state.heading = math.radians(90 - imu_yaw_degrees)
        else:
            # Estimate heading from differential drive
            v_angular = (v_right - v_left) / self.WHEEL_BASE * self.turn_calibration
            self.state.heading += v_angular * dt

        # Normalize heading to -π to π
        self.state.heading = (self.state.heading + math.pi) % (2 * math.pi) - math.pi

        # Update position using current heading
        self.state.x += v_linear * math.cos(self.state.heading) * dt
        self.state.y += v_linear * math.sin(self.state.heading) * dt

        # Update velocity
        self.state.velocity = v_linear

        # Update timestamp
        self.state.last_update = current_time

        logger.debug(
            f"Odometry: pos=({self.state.x:.2f}, {self.state.y:.2f}), "
            f"heading={math.degrees(self.state.heading):.1f}°, "
            f"vel={self.state.velocity:.2f}m/s"
        )

    def update_from_imu(self, yaw_degrees: float):
        """
        Update heading from IMU reading only.

        Args:
            yaw_degrees: IMU yaw angle in degrees
        """
        # Convert IMU yaw to our heading convention
        self.state.heading = math.radians(90 - yaw_degrees)
        self.state.heading = (self.state.heading + math.pi) % (2 * math.pi) - math.pi

    def get_position(self) -> Tuple[float, float, float]:
        """
        Get current position.

        Returns:
            (x, y, heading) tuple where heading is in radians
        """
        return (self.state.x, self.state.y, self.state.heading)

    def get_position_tuple(self) -> Tuple[float, float]:
        """Get current (x, y) position."""
        return (self.state.x, self.state.y)

    def get_heading_degrees(self) -> float:
        """Get current heading in degrees."""
        return math.degrees(self.state.heading)

    def get_velocity(self) -> float:
        """Get current velocity in m/s."""
        return self.state.velocity

    def reset(self, x: float = 0.0, y: float = 0.0, heading: float = 0.0):
        """
        Reset odometry to a specific position.

        Args:
            x: X position in meters
            y: Y position in meters
            heading: Heading in radians
        """
        self.state.x = x
        self.state.y = y
        self.state.heading = heading
        self.state.velocity = 0.0
        self.state.last_update = time.time()
        logger.info(f"Odometry reset to ({x:.2f}, {y:.2f}), {math.degrees(heading):.1f}°")

    def calibrate_speed(self, actual_distance: float, commanded_distance: float):
        """
        Calibrate speed estimation.

        Args:
            actual_distance: Actual distance traveled (measured)
            commanded_distance: Commanded distance (calculated from odometry)
        """
        if commanded_distance > 0:
            self.speed_calibration = actual_distance / commanded_distance
            logger.info(f"Speed calibration updated: {self.speed_calibration:.3f}")

    def calibrate_turn_rate(self, actual_angle: float, commanded_angle: float):
        """
        Calibrate turning rate.

        Args:
            actual_angle: Actual angle turned (measured in radians)
            commanded_angle: Commanded angle (calculated from odometry)
        """
        if commanded_angle != 0:
            self.turn_calibration = actual_angle / commanded_angle
            logger.info(f"Turn calibration updated: {self.turn_calibration:.3f}")

    def get_distance_traveled(self) -> float:
        """Get total distance traveled from origin."""
        return math.sqrt(self.state.x**2 + self.state.y**2)

    def get_distance_to_point(self, target_x: float, target_y: float) -> float:
        """
        Calculate distance to a target point.

        Args:
            target_x: Target X coordinate
            target_y: Target Y coordinate

        Returns:
            Distance in meters
        """
        dx = target_x - self.state.x
        dy = target_y - self.state.y
        return math.sqrt(dx**2 + dy**2)

    def get_heading_to_point(self, target_x: float, target_y: float) -> float:
        """
        Calculate required heading to reach a target point.

        Args:
            target_x: Target X coordinate
            target_y: Target Y coordinate

        Returns:
            Heading in radians
        """
        dx = target_x - self.state.x
        dy = target_y - self.state.y
        return math.atan2(dy, dx)

    def is_near_waypoint(self, target_x: float, target_y: float, threshold: float = 0.1) -> bool:
        """
        Check if robot is near a waypoint.

        Args:
            target_x: Target X coordinate
            target_y: Target Y coordinate
            threshold: Distance threshold in meters

        Returns:
            True if within threshold distance
        """
        return self.get_distance_to_point(target_x, target_y) < threshold


# Global instance
odometry = Odometry()
