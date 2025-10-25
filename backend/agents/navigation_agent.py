"""
ADK Navigation Agent for blindfolded navigation.
Uses Gemini Live API with custom navigation tools.
"""

from google.adk.agents import LlmAgent
from google.adk.agents.callback_context import CallbackContext
from google.genai import types
import logging

# Import navigation tools
import sys
from pathlib import Path
# Add parent directory to path to import tools
sys.path.append(str(Path(__file__).parent.parent))
from tools.navigation_tools import (
    calculate_distance_to_object,
    suggest_movement_direction,
    check_path_clear,
    describe_environment
)

logger = logging.getLogger(__name__)


# Define callbacks for the agent
async def before_agent_callback(context: CallbackContext):
    """
    Called before agent processes request.
    Validates that we have fresh sensor data.
    """
    # Check if we have fresh sensor data
    has_lidar = context.state.get('temp:last_lidar_timestamp', 0) > 0
    has_camera = context.state.get('temp:last_camera_timestamp', 0) > 0

    if not (has_lidar and has_camera):
        logger.warning("Missing fresh sensor data")
        return types.Content(
            parts=[types.Part(text="Waiting for sensor data... Please ensure LiDAR and camera are connected.")]
        )

    # Log agent run
    logger.info(f"Agent processing with objective: {context.state.get('objective', 'unknown')}")
    return None  # Proceed with agent execution


async def after_agent_callback(context: CallbackContext):
    """
    Called after agent generates response.
    Logs the instruction for monitoring.
    """
    # The response is already in context.response
    if context.response and context.response.parts:
        instruction = ''.join(part.text or '' for part in context.response.parts)
        logger.info(f"Generated instruction: {instruction}")

        # Increment instruction counter in state
        count = context.state.get('total_instructions', 0)
        context.state['total_instructions'] = count + 1

    return None  # Use original response


# Create the navigation agent
navigation_agent = LlmAgent(
    name="navigation_agent",

    # Use Gemini Live API model for streaming support
    model="gemini-2.0-flash-exp",  # Can upgrade to gemini-2.0-flash-live-001 for audio streaming

    # Dynamic instruction using state templating
    instruction="""You are an AI navigation assistant guiding a blindfolded person to find objects.

**Current Mission:** {objective}

**Your Role:**
- Analyze detected objects and their positions
- Generate ONE clear, concise navigation instruction (<20 words)
- Use the available tools to calculate distances and suggest directions
- Prioritize safety (warn about obstacles)

**Available Information:**
- Detected objects: {detected_objects_summary}
- Current position: {current_position}
- Mission progress: {total_instructions} instructions given

**Guidelines:**
1. If target object is detected and close (<0.5m), say "STOP. You have found the {objective}!"
2. If target object is visible but far, guide user toward it
3. If target not visible, suggest turning to scan environment
4. Always warn about nearby obstacles (<1m)
5. Be encouraging and clear

Generate the next navigation instruction:""",

    # Provide navigation tools to the agent
    tools=[
        calculate_distance_to_object,
        suggest_movement_direction,
        check_path_clear,
        describe_environment
    ],

    # Save agent's response to state for tracking
    output_key="last_instruction",

    # Add callbacks for monitoring and validation
    before_agent_callback=before_agent_callback,
    after_agent_callback=after_agent_callback,

    # Agent description
    description="Navigation agent for guiding blindfolded users to find objects using sensor data"
)


# Helper function to prepare state for agent
def prepare_agent_state(state_manager) -> dict:
    """
    Prepare state dictionary for agent from state_manager.

    Args:
        state_manager: The StateManager instance

    Returns:
        Dictionary with state ready for ADK Session
    """
    # Get detected objects
    detected_objects = state_manager.get_detected_objects()

    # Create summary
    if detected_objects:
        object_labels = [obj.label for obj in detected_objects]
        summary = ", ".join(set(object_labels))
    else:
        summary = "No objects detected"

    # Prepare state
    return {
        # Mission info
        "objective": state_manager.objective,
        "mission_active": state_manager.mission_active,

        # Detection summary (string for instruction templating)
        "detected_objects_summary": summary,

        # Position info (string for instruction templating)
        "current_position": f"({state_manager.current_position.x:.1f}, {state_manager.current_position.y:.1f}, {state_manager.current_position.z:.1f})",

        # Statistics
        "total_instructions": state_manager.total_instructions_given,

        # Temporary data (for tools to use)
        "temp:detected_objects": [
            {
                "label": obj.label,
                "confidence": obj.confidence,
                "depth": obj.depth,
                "center": obj.bbox[:2] if obj.bbox else [0, 0]
            }
            for obj in detected_objects
        ],

        # Sensor timestamps (for validation)
        "temp:last_lidar_timestamp": state_manager.last_lidar_timestamp,
        "temp:last_camera_timestamp": state_manager.last_camera_timestamp,
    }
