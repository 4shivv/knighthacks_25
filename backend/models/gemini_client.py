"""
Gemini API client wrapper for AI planning and reasoning.
Handles prompt building, API calls, retries, and error handling.
"""

import google.genai as genai
from google.genai import types
from typing import Optional, Dict, Any, List
import logging
import time

logger = logging.getLogger(__name__)


class GeminiClient:
    """
    Wrapper for Google Gemini API.
    Handles planning/reasoning requests with retry logic and error handling.
    """

    def __init__(self, api_key: str, model_name: str = "gemini-2.0-flash-exp"):
        """
        Initialize Gemini client.

        Args:
            api_key: Google AI API key
            model_name: Model to use (default: gemini-2.0-flash-exp for speed)
        """
        self.api_key = api_key
        self.model_name = model_name
        self.client: Optional[genai.Client] = None
        self._initialize_client()

    def _initialize_client(self):
        """Initialize the Gemini API client."""
        try:
            logger.info(f"Initializing Gemini client with model {self.model_name}...")
            self.client = genai.Client(api_key=self.api_key)
            logger.info("Gemini client initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize Gemini client: {e}")
            raise

    def generate_navigation_instruction(
        self,
        objective: str,
        detected_objects: List[Dict[str, Any]],
        position_history: List[Dict[str, float]],
        current_position: Dict[str, float],
        max_retries: int = 3
    ) -> Optional[str]:
        """
        Generate a navigation instruction based on current context.

        Args:
            objective: Mission objective (e.g., "Find a red coffee mug")
            detected_objects: List of currently detected objects with positions
            position_history: Recent position history
            current_position: Current position {x, y, z}
            max_retries: Number of retry attempts on failure

        Returns:
            Navigation instruction string (short, <20 words) or None on failure
        """
        if self.client is None:
            logger.error("Gemini client not initialized")
            return None

        # Build context prompt
        prompt = self._build_navigation_prompt(
            objective,
            detected_objects,
            position_history,
            current_position
        )

        # Try to generate with retries
        for attempt in range(max_retries):
            try:
                response = self.client.models.generate_content(
                    model=self.model_name,
                    contents=prompt,
                    config=types.GenerateContentConfig(
                        temperature=0.3,  # Lower temperature for more deterministic output
                        max_output_tokens=100,  # Keep responses short
                    )
                )

                instruction = response.text.strip()

                # Validate instruction length
                if len(instruction.split()) > 25:
                    logger.warning("Instruction too long, truncating...")
                    words = instruction.split()[:20]
                    instruction = " ".join(words)

                logger.info(f"Generated instruction: {instruction}")
                return instruction

            except Exception as e:
                logger.error(f"Gemini API error (attempt {attempt + 1}/{max_retries}): {e}")
                if attempt < max_retries - 1:
                    time.sleep(1)  # Wait before retry
                else:
                    logger.error("All retry attempts failed")
                    return self._get_fallback_instruction()

        return None

    def _build_navigation_prompt(
        self,
        objective: str,
        detected_objects: List[Dict[str, Any]],
        position_history: List[Dict[str, float]],
        current_position: Dict[str, float]
    ) -> str:
        """Build a detailed context prompt for Gemini."""

        # Format detected objects
        objects_str = ""
        if detected_objects:
            objects_list = []
            for obj in detected_objects:
                depth_str = f"{obj.get('depth', 'unknown')}m" if obj.get('depth') else "unknown distance"
                objects_list.append(f"- {obj['label']} (confidence: {obj['confidence']:.2f}, distance: {depth_str})")
            objects_str = "\n".join(objects_list)
        else:
            objects_str = "No objects currently detected"

        # Format position history
        position_str = f"Current: ({current_position['x']:.1f}, {current_position['y']:.1f}, {current_position['z']:.1f})"
        if len(position_history) > 1:
            last_positions = position_history[-5:]  # Last 5 positions
            history_str = " → ".join([f"({p['x']:.1f}, {p['y']:.1f})" for p in last_positions])
            position_str += f"\nRecent path: {history_str}"

        # Build full prompt
        prompt = f"""You are guiding a blindfolded person through a room using sensor data.

OBJECTIVE: {objective}

CURRENT SITUATION:
{position_str}

VISIBLE OBJECTS:
{objects_str}

TASK:
Generate ONE SHORT navigation instruction to help them reach their objective.
The instruction must be:
- Clear and specific (e.g., "Move 2 feet forward")
- Under 20 words
- Action-oriented
- Based on the detected objects and current position

If the objective object is detected and close (<1m), say "STOP. You have found the [object]!"

INSTRUCTION:"""

        return prompt

    def _get_fallback_instruction(self) -> str:
        """Get a safe fallback instruction when API fails."""
        fallback_instructions = [
            "Move forward cautiously",
            "Stop and wait for new sensor data",
            "Turn slowly in place to scan surroundings"
        ]
        return fallback_instructions[0]

    def analyze_scene(
        self,
        detected_objects: List[Dict[str, Any]],
        objective: str
    ) -> Dict[str, Any]:
        """
        Analyze the current scene and provide context.

        Args:
            detected_objects: List of detected objects
            objective: Mission objective

        Returns:
            Analysis dict with recommendations
        """
        if self.client is None:
            logger.error("Gemini client not initialized")
            return {"error": "Client not initialized"}

        # Build analysis prompt
        objects_str = ", ".join([f"{obj['label']} ({obj['confidence']:.2f})" for obj in detected_objects])

        prompt = f"""Analyze this scene:
Objective: {objective}
Detected objects: {objects_str}

Provide:
1. Is the target object visible? (yes/no)
2. What's the closest relevant object?
3. Recommended next action (one sentence)

Be concise."""

        try:
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=prompt,
                config=types.GenerateContentConfig(
                    temperature=0.2,
                    max_output_tokens=150
                )
            )

            return {
                "analysis": response.text.strip(),
                "detected_count": len(detected_objects)
            }

        except Exception as e:
            logger.error(f"Scene analysis error: {e}")
            return {"error": str(e)}

    def test_connection(self) -> bool:
        """Test if the Gemini API connection works."""
        try:
            response = self.client.models.generate_content(
                model=self.model_name,
                contents="Say 'OK' if you can read this.",
                config=types.GenerateContentConfig(max_output_tokens=10)
            )
            result = "ok" in response.text.lower()
            logger.info(f"Gemini connection test: {'PASS' if result else 'FAIL'}")
            return result
        except Exception as e:
            logger.error(f"Gemini connection test failed: {e}")
            return False

    def get_client_info(self) -> Dict[str, Any]:
        """Get information about the client."""
        return {
            "initialized": self.client is not None,
            "model_name": self.model_name,
            "api_key_set": bool(self.api_key)
        }
