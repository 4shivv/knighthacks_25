"""
Planning/Reasoning Loop (runs at 1 Hz)
Uses ADK agent to generate navigation instructions based on sensor data.
"""

import asyncio
import logging
from typing import Optional

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

                # STEP 5.2: Build context for ADK agent
                # Prepare agent state from state manager
                from agents.navigation_agent import prepare_agent_state
                agent_state = prepare_agent_state(self.state_manager)

                # STEP 5.3: Update ADK session state
                await self.adk_session_manager.update_session_state(agent_state)

                # STEP 5.4: Call ADK agent (this includes Gemini API call)
                instruction = await self.adk_session_manager.run_agent(
                    query="Based on current sensor data, generate the next navigation instruction."
                )

                # STEP 5.5: Parse and store instruction
                if instruction and instruction != "No instruction generated":
                    self.state_manager.add_instruction(instruction)
                    logger.info(f"Generated instruction: {instruction}")

                    # STEP 5.6: Log planning result
                    logger.debug(f"Planning cycle complete. Instruction queued.")
                else:
                    logger.warning("Agent did not generate instruction")

            except Exception as e:
                logger.error(f"Error in planning loop: {e}", exc_info=True)

            # Wait for next cycle
            await asyncio.sleep(self.interval_seconds)

        logger.info("Planning loop stopped")

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
    print(f"\nGenerated {state_manager.total_instructions_given} instructions")
    print(f"Instruction queue: {state_manager.instruction_queue}")

    # Clean up
    await adk_manager.end_session()


if __name__ == "__main__":
    # Run test
    asyncio.run(test_planning_loop())
