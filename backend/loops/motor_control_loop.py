"""
Motor Control Loop for ELEGOO Smart Robot Car V4.0
Executes motor commands from queue via WiFi bridge to Arduino.

Runs at 0.3 Hz (every 3 seconds + execution time)
"""

import asyncio
import logging
import time
import json
from typing import Optional
from fastapi import WebSocket

logger = logging.getLogger(__name__)


class MotorControlLoop:
    """
    Motor control loop that executes queued motor commands.
    Sends commands to Arduino via WiFi WebSocket bridge.
    """

    def __init__(
        self,
        state_manager,
        odometry,
        interval_ms: int = 3000
    ):
        """
        Initialize motor control loop.

        Args:
            state_manager: StateManager instance
            odometry: Odometry instance
            interval_ms: Loop interval in milliseconds
        """
        self.state_manager = state_manager
        self.odometry = odometry
        self.interval_seconds = interval_ms / 1000.0
        self.running = False

        # Arduino WebSocket connection (will be set by main.py)
        self.arduino_ws: Optional[WebSocket] = None

        # Statistics
        self.total_commands_sent = 0
        self.total_acks_received = 0
        self.last_command_time = 0

        logger.info(f"Motor control loop initialized (interval: {interval_ms}ms)")

    def set_arduino_websocket(self, websocket: WebSocket):
        """
        Set the Arduino WebSocket connection.

        Args:
            websocket: WebSocket connection to Arduino
        """
        self.arduino_ws = websocket
        logger.info("Arduino WebSocket connection established")

    async def run(self):
        """Main motor control loop - runs continuously."""
        self.running = True
        logger.info("Motor control loop started")

        # Import here to avoid circular imports
        from tools.motor_command_translator import motor_translator

        while self.running:
            try:
                # Check if Arduino is connected
                if not self.arduino_ws:
                    logger.debug("Waiting for Arduino WiFi connection...")
                    await asyncio.sleep(self.interval_seconds)
                    continue

                # Pop next command from queue
                command = self.state_manager.pop_motor_command()

                if command:
                    # Get current position for waypoint calculations
                    current_pos = self.odometry.get_position_tuple()
                    current_heading = self.odometry.state.heading

                    # Translate high-level command to ELEGOO JSON format
                    motor_cmd = motor_translator.translate(
                        command,
                        current_position=current_pos,
                        current_heading=current_heading
                    )

                    # Send to Arduino via WiFi WebSocket
                    success = await self._send_motor_command(motor_cmd)

                    if success:
                        self.total_commands_sent += 1
                        self.last_command_time = time.time()

                        # Update odometry from motor command
                        await self._update_odometry_from_command(motor_cmd)

                        logger.info(
                            f"Motor command sent #{self.total_commands_sent}: "
                            f"L={motor_cmd.get('D1', 0)}, R={motor_cmd.get('D2', 0)}"
                        )
                    else:
                        logger.error("Failed to send motor command - Arduino may be disconnected")

                else:
                    # No command in queue: send STOP to ensure motors are idle
                    await self._send_stop_command()

                # Wait for next cycle
                await asyncio.sleep(self.interval_seconds)

            except Exception as e:
                logger.error(f"Error in motor control loop: {e}", exc_info=True)
                await asyncio.sleep(self.interval_seconds)

        logger.info("Motor control loop stopped")

    async def _send_motor_command(self, motor_cmd: dict, max_retries: int = 3) -> bool:
        """
        Send motor command to Arduino via WebSocket with retry logic.

        Args:
            motor_cmd: ELEGOO JSON command dict
            max_retries: Maximum number of retry attempts

        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.arduino_ws:
            return False

        for attempt in range(max_retries):
            try:
                # Send JSON command to Arduino
                await self.arduino_ws.send_json(motor_cmd)

                # Wait for acknowledgment (with timeout)
                ack_received = await self._wait_for_ack(
                    motor_cmd.get("H", "000"),
                    timeout=1.0
                )

                if ack_received:
                    self.total_acks_received += 1
                    return True
                else:
                    logger.warning(
                        f"No ack for command {motor_cmd.get('H')} (attempt {attempt + 1}/{max_retries})"
                    )

            except Exception as e:
                logger.error(f"Error sending motor command (attempt {attempt + 1}): {e}")

            # Small delay before retry
            if attempt < max_retries - 1:
                await asyncio.sleep(0.1)

        return False

    async def _send_stop_command(self):
        """Send STOP command to ensure motors are idle."""
        stop_cmd = {
            "N": 4,
            "H": "000",
            "D1": 0,
            "D2": 0
        }

        # Only send if no command was sent recently (avoid spam)
        if time.time() - self.last_command_time > 2.0:
            try:
                if self.arduino_ws:
                    await self.arduino_ws.send_json(stop_cmd)
                    logger.debug("STOP command sent (idle)")
            except Exception as e:
                logger.error(f"Error sending STOP command: {e}")

    async def _wait_for_ack(self, command_id: str, timeout: float = 1.0) -> bool:
        """
        Wait for acknowledgment from Arduino.

        Args:
            command_id: Command ID to wait for
            timeout: Timeout in seconds

        Returns:
            True if ack received, False if timeout
        """
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            # Check if ack was received (stored in state manager by Arduino WebSocket handler)
            ack = self.state_manager.last_motor_ack if hasattr(self.state_manager, 'last_motor_ack') else None

            if ack and ack.get("command_id") == command_id and ack.get("status") == "ok":
                return True

            await asyncio.sleep(0.05)

        return False

    async def _update_odometry_from_command(self, motor_cmd: dict):
        """
        Update odometry based on motor command.

        Args:
            motor_cmd: ELEGOO motor command dict
        """
        try:
            left_speed = motor_cmd.get("D1", 0)
            right_speed = motor_cmd.get("D2", 0)

            # Get IMU yaw if available from state manager
            imu_yaw = None
            if hasattr(self.state_manager, 'current_heading') and self.state_manager.current_heading != 0:
                imu_yaw = self.state_manager.current_heading

            # Update odometry
            self.odometry.update_from_motor_commands(
                left_speed_pwm=left_speed,
                right_speed_pwm=right_speed,
                imu_yaw_degrees=imu_yaw
            )

            # Update state manager with new position
            new_pos = self.odometry.get_position()
            from state_manager import Position
            self.state_manager.update_position(
                Position(x=new_pos[0], y=new_pos[1], z=0.0)
            )

            # Update heading and velocity in state manager
            self.state_manager.update_odometry(
                heading=self.odometry.get_heading_degrees(),
                velocity=self.odometry.get_velocity()
            )

        except Exception as e:
            logger.error(f"Error updating odometry: {e}")

    def get_stats(self) -> dict:
        """Get motor control loop statistics."""
        return {
            "running": self.running,
            "commands_sent": self.total_commands_sent,
            "acks_received": self.total_acks_received,
            "last_command_time": self.last_command_time,
            "arduino_connected": self.arduino_ws is not None
        }


# Helper function to stop motors immediately (emergency)
async def emergency_stop_motors(arduino_ws: WebSocket):
    """
    Emergency stop - send STOP command immediately.

    Args:
        arduino_ws: Arduino WebSocket connection
    """
    if arduino_ws:
        try:
            stop_cmd = {
                "N": 4,
                "H": "999",
                "D1": 0,
                "D2": 0
            }
            await arduino_ws.send_json(stop_cmd)
            logger.warning("EMERGENCY STOP sent to Arduino!")
        except Exception as e:
            logger.error(f"Failed to send emergency stop: {e}")
