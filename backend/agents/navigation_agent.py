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
    # Legacy tools (still useful)
    calculate_distance_to_object,
    check_path_clear,
    describe_environment,
    # New D* Lite integration tools
    check_goal_reached,
    validate_path_safety,
    calculate_motor_command_for_waypoint,
    get_exploration_command
)

logger = logging.getLogger(__name__)


# Define callbacks for the agent
async def before_agent_callback(**kwargs):
    """
    Called before agent processes request.
    Validates that we have fresh sensor data.
    """
    context = kwargs.get('callback_context')
    if not context:
        return None

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


async def after_agent_callback(**kwargs):
    """
    Called after agent generates response.
    Logs the instruction for monitoring.
    """
    context = kwargs.get('callback_context')
    if not context:
        return None

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

    # Dynamic instruction using state templating (updated for RC car autonomous navigation)
    instruction="""You are an autonomous navigation decision system for an RC car.

**Current Mission:** {objective}

**Your Role:**
- Monitor D* Lite pathplanner status and make high-level navigation decisions
- Override pathplanner when safety requires immediate action
- Generate structured motor commands or path-following directives
- Decide between: following planned path, emergency stop, exploration, or replanning

**Current Status:**
- Position: {current_position}
- Heading: {current_heading}
- Velocity: {velocity}
- Detected objects: {detected_objects_summary}
- Path status: {path_status}
- Next waypoint: {next_waypoint}
- Goal position: {goal_position}
- Path length: {current_path_length} steps
- Obstacles in path: {obstacles_in_path}
- Commands sent: {motor_commands_sent}
- D* Lite replans: {dstar_replans}

**Decision Logic (choose ONE):**

1. **MISSION_COMPLETE** - Target found and reached
   - Condition: Target detected, confidence >0.95, depth <0.5m
   - Action: Return motor command: {{"command": "STOP", "reason": "MISSION_COMPLETE"}}

2. **FOLLOW_PATH** - Trust D* Lite pathplanner
   - Condition: path_status="PATH_FOUND", no unexpected obstacles, waypoint available
   - Action: Return motor command: {{"command": "MOVE_TO_WAYPOINT", "waypoint": <next_waypoint>, "speed": 60}}

3. **EMERGENCY_STOP** - Immediate safety concern
   - Condition: Unexpected obstacle <0.3m ahead, sensor data stale, or path blocked
   - Action: Return motor command: {{"command": "EMERGENCY_STOP", "reason": "<why>"}}

4. **REQUEST_REPLAN** - Path invalidated by new obstacles
   - Condition: path_status="BLOCKED" or obstacles detected in current path
   - Action: Return motor command: {{"command": "REQUEST_REPLAN", "reason": "Obstacles detected"}}

5. **EXPLORE** - No goal set yet, systematic search
   - Condition: path_status="NO_GOAL", target not visible
   - Action: Return motor command: {{"command": "ROTATE_SCAN", "angle": 45, "speed": 30}}

6. **BACKUP_RECOVERY** - RC car stuck or no progress
   - Condition: Velocity near 0 for multiple cycles, path available but not moving
   - Action: Return motor command: {{"command": "BACKUP", "distance": 0.3, "speed": 40}}

**Output Format:**
Return ONLY a valid JSON motor command object. Do not include explanations or additional text.

Example outputs:
{{"command": "MOVE_TO_WAYPOINT", "waypoint": [2.3, 4.1], "speed": 60}}
{{"command": "EMERGENCY_STOP", "reason": "Obstacle detected at 0.2m"}}
{{"command": "STOP", "reason": "MISSION_COMPLETE"}}

Generate motor command:""",

    # Provide navigation tools to the agent
    tools=[
        # Core detection tools
        calculate_distance_to_object,
        check_path_clear,
        describe_environment,
        # D* Lite integration tools
        check_goal_reached,
        validate_path_safety,
        calculate_motor_command_for_waypoint,
        get_exploration_command
    ],

    # Save agent's response to state for tracking
    output_key="last_instruction",

    # Add callbacks for monitoring and validation
    before_agent_callback=before_agent_callback,
    after_agent_callback=after_agent_callback,

    # Agent description
    description="Autonomous navigation decision system for RC car pathfinding and obstacle avoidance"
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

    # Format pathfinding state
    next_waypoint_str = f"({state_manager.next_waypoint[0]:.1f}, {state_manager.next_waypoint[1]:.1f})" if state_manager.next_waypoint else "None"
    goal_position_str = f"Grid: {state_manager.goal_position}" if state_manager.goal_position else "Not set"

    # Count obstacles in current path (if path exists)
    obstacles_in_path = 0
    if state_manager.current_path and detected_objects:
        # Simple heuristic: count objects with depth < 1m that might be in the path
        obstacles_in_path = sum(1 for obj in detected_objects if obj.depth and obj.depth < 1.0)

    # Prepare state
    return {
        # Mission info
        "objective": state_manager.objective,
        "mission_active": state_manager.mission_active,

        # Detection summary (string for instruction templating)
        "detected_objects_summary": summary,

        # Position info (string for instruction templating)
        "current_position": f"({state_manager.current_position.x:.1f}, {state_manager.current_position.y:.1f}, {state_manager.current_position.z:.1f})",

        # Pathfinding context (for RC car navigation)
        "next_waypoint": next_waypoint_str,
        "path_status": state_manager.path_status,
        "current_path_length": len(state_manager.current_path),
        "goal_position": goal_position_str,
        "obstacles_in_path": obstacles_in_path,

        # Movement context
        "current_heading": f"{state_manager.current_heading:.1f}°",
        "velocity": f"{state_manager.velocity:.2f} m/s",
        "last_motor_command": str(state_manager.last_motor_command) if state_manager.last_motor_command else "None",

        # Statistics
        "motor_commands_sent": state_manager.total_motor_commands_sent,
        "dstar_replans": state_manager.dstar_replan_count,

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
        "temp:current_path": state_manager.current_path,
        "temp:occupancy_grid_available": state_manager.occupancy_grid is not None,

        # Sensor timestamps (for validation)
        "temp:last_lidar_timestamp": state_manager.last_lidar_timestamp,
        "temp:last_camera_timestamp": state_manager.last_camera_timestamp,
    }
