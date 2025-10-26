"""
Planning/Reasoning Loop (runs at 1 Hz)
Uses ADK agent to generate motor commands for RC car navigation based on sensor data and D* Lite pathfinding.
"""

import asyncio
import logging
import json
from typing import Optional
from config import settings
from agents.navigation_agent import prepare_agent_state

logger = logging.getLogger(__name__)


class PlanningLoop:
    """
    Planning loop that runs the ADK navigation agent to generate instructions.
    """

    def __init__(
        self,
        state_manager,
        adk_session_manager,
        interval_ms: int = 1000
    ):
        """
        Initialize planning loop.

        Args:
            state_manager: StateManager instance
            adk_session_manager: ADKSessionManager instance
            interval_ms: Loop interval in milliseconds
        """
        self.state_manager = state_manager
        self.adk_session_manager = adk_session_manager
        self.interval_seconds = interval_ms / 1000.0
        self.running = False
        self.task: Optional[asyncio.Task] = None

        logger.info(f"Planning loop initialized (interval: {interval_ms}ms)")

    async def run(self):
        """Main planning loop - runs continuously."""
        self.running = True
        logger.info("Planning loop started")

        while self.running:
            try:
                # Only run if mission is active
                if not self.state_manager.mission_active:
                    await asyncio.sleep(self.interval_seconds)
                    continue

                # Check if mission is already complete
                if self.state_manager.mission_complete:
                    logger.info("Mission complete - planning loop paused")
                    await asyncio.sleep(self.interval_seconds)
                    continue

                # STEP 5.1: Retrieve latest state
                detected_objects = self.state_manager.get_detected_objects()
                position_history = self.state_manager.position_history
                current_position = self.state_manager.current_position

                logger.debug(f"Planning with {len(detected_objects)} detected objects")

                # STEP 5.1a: Check for mission success (deterministic)
                success_result = self._check_mission_success(detected_objects)
                if success_result['mission_complete']:
                    logger.info(f"Mission success detected: {success_result['message']}")
                    # Mark mission complete
                    self.state_manager.complete_mission(found_object=success_result.get('found_object'))
                    # Send STOP command
                    stop_command = {
                        "command": "STOP",
                        "reason": "MISSION_COMPLETE",
                        "found_object": success_result['object_label'],
                        "distance": success_result['distance'],
                        "confidence": success_result['confidence']
                    }
                    self.state_manager.add_motor_command(stop_command)
                    logger.info(f"STOP command issued: {stop_command}")
                    # Skip rest of planning cycle
                    await asyncio.sleep(self.interval_seconds)
                    continue

                # STEP 5.2: Build context for ADK agent
                # Prepare agent state from state manager (imported at top of file)
                agent_state = prepare_agent_state(self.state_manager)

                # STEP 5.3: Update ADK session state
                await self.adk_session_manager.update_session_state(agent_state)

                # STEP 5.4: Call ADK agent (this includes Gemini API call)
                # Agent now returns JSON motor command instead of text instruction
                agent_response = await self.adk_session_manager.run_agent(
                    query="Based on current sensor data and pathfinding status, generate the next motor command."
                )

                # STEP 5.5: Parse and store motor command
                if agent_response and agent_response != "No instruction generated":
                    # Try to parse JSON motor command from agent response
                    motor_command = self._parse_motor_command(agent_response)

                    if motor_command:
                        # Store motor command in queue
                        self.state_manager.add_motor_command(motor_command)
                        logger.info(f"Generated motor command: {motor_command.get('command', 'UNKNOWN')}")

                        # STEP 5.6: Log planning result
                        logger.debug(f"Planning cycle complete. Motor command queued.")
                    else:
                        logger.warning(f"Failed to parse motor command from agent response: {agent_response}")
                else:
                    logger.warning("Agent did not generate response")

            except Exception as e:
                logger.error(f"Error in planning loop: {e}", exc_info=True)

            # Wait for next cycle
            await asyncio.sleep(self.interval_seconds)

        logger.info("Planning loop stopped")

    def _check_mission_success(self, detected_objects: list) -> dict:
        """
        Deterministic check for mission success.
        Returns mission complete if target object is detected with high confidence and close distance.

        Args:
            detected_objects: List of DetectedObject instances

        Returns:
            Dictionary with success status and details
        """
        # Extract target object label from mission objective (settings imported at top)
        objective = self.state_manager.objective.lower()

        # Simple keyword extraction (e.g., "Find a red coffee mug" -> look for "mug" or "cup")
        # This could be enhanced with NLP
        target_keywords = []
        if "mug" in objective or "cup" in objective:
            target_keywords = ["mug", "cup"]
        elif "bottle" in objective:
            target_keywords = ["bottle"]
        elif "chair" in objective:
            target_keywords = ["chair"]
        elif "person" in objective:
            target_keywords = ["person"]
        else:
            # Extract nouns as fallback
            words = objective.split()
            target_keywords = [w for w in words if len(w) > 3 and w not in ['find', 'locate', 'search']]

        if not target_keywords:
            return {"mission_complete": False, "reason": "No target keywords identified"}

        # Check each detected object
        for obj in detected_objects:
            obj_label = obj.label.lower()

            # Check if this object matches target
            if any(keyword in obj_label or obj_label in keyword for keyword in target_keywords):
                # Check success criteria
                if (obj.confidence >= settings.success_confidence_threshold and
                    obj.depth is not None and
                    obj.depth <= settings.success_distance_threshold):

                    return {
                        "mission_complete": True,
                        "object_label": obj.label,
                        "confidence": obj.confidence,
                        "distance": obj.depth,
                        "found_object": obj,
                        "message": f"Target found: {obj.label} at {obj.depth:.2f}m with {obj.confidence:.2%} confidence"
                    }

        return {"mission_complete": False, "reason": "Target not found or not close enough"}

    def _parse_motor_command(self, agent_response: str) -> Optional[dict]:
        """
        Parse motor command from agent response.
        Agent should return JSON, but may wrap it in markdown or text.

        Args:
            agent_response: Raw agent response

        Returns:
            Parsed motor command dict or None if parsing fails
        """
        try:
            # Try direct JSON parse first
            try:
                return json.loads(agent_response)
            except json.JSONDecodeError:
                pass

            # Try extracting JSON from markdown code block
            if "```json" in agent_response:
                start = agent_response.find("```json") + 7
                end = agent_response.find("```", start)
                json_str = agent_response[start:end].strip()
                return json.loads(json_str)
            elif "```" in agent_response:
                start = agent_response.find("```") + 3
                end = agent_response.find("```", start)
                json_str = agent_response[start:end].strip()
                return json.loads(json_str)

            # Try finding JSON object pattern
            import re
            json_pattern = r'\{[^}]+\}'
            matches = re.findall(json_pattern, agent_response, re.DOTALL)
            if matches:
                # Try each match
                for match in matches:
                    try:
                        parsed = json.loads(match)
                        if 'command' in parsed:  # Validate it looks like a motor command
                            return parsed
                    except json.JSONDecodeError:
                        continue

            # If all parsing fails, log and return None
            logger.warning(f"Could not parse motor command from: {agent_response[:100]}...")
            return None

        except Exception as e:
            logger.error(f"Error parsing motor command: {e}")
            return None

    def start(self):
        """Start the planning loop as a background task."""
        if self.running:
            logger.warning("Planning loop already running")
            return

        self.task = asyncio.create_task(self.run())
        logger.info("Planning loop task created")

    async def stop(self):
        """Stop the planning loop."""
        self.running = False
        if self.task:
            await self.task
            logger.info("Planning loop stopped gracefully")


# Standalone function for testing
async def test_planning_loop():
    """Test the planning loop with mock data."""
    from state_manager import state_manager
    from agents.navigation_agent import navigation_agent
    from adk_session_manager import ADKSessionManager

    # Initialize components
    adk_manager = ADKSessionManager(navigation_agent)

    # Start mission
    state_manager.start_mission("Find a red coffee mug")

    # Create initial session
    from agents.navigation_agent import prepare_agent_state
    initial_state = prepare_agent_state(state_manager)
    await adk_manager.start_new_session(initial_state)

    # Add some mock detections
    from state_manager import DetectedObject
    import time

    mock_objects = [
        DetectedObject(
            label="cup",
            confidence=0.85,
            bbox=[100, 100, 50, 50],
            depth=2.3
        ),
        DetectedObject(
            label="chair",
            confidence=0.92,
            bbox=[300, 200, 80, 100],
            depth=1.5
        )
    ]

    for obj in mock_objects:
        state_manager.add_detected_object(obj)

    # Set sensor timestamps
    state_manager.last_lidar_timestamp = time.time()
    state_manager.last_camera_timestamp = time.time()

    # Create and run planning loop
    planning_loop = PlanningLoop(
        state_manager=state_manager,
        adk_session_manager=adk_manager,
        interval_ms=2000  # 2 seconds for testing
    )

    # Run for a few cycles
    planning_loop.start()
    await asyncio.sleep(6)  # Run 3 cycles
    await planning_loop.stop()

    # Check results
    print(f"\nGenerated {state_manager.total_motor_commands_sent} motor commands")
    print(f"Motor command queue: {state_manager.motor_command_queue}")

    # Clean up
    await adk_manager.end_session()


if __name__ == "__main__":
    # Run test
    asyncio.run(test_planning_loop())
