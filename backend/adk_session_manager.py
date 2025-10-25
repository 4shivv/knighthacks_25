"""
ADK Session Manager - Wrapper to integrate ADK Sessions with our StateManager.
Bridges our custom StateManager with ADK's Session/State system.
"""

from google.adk.sessions import InMemorySessionService, Session
from google.adk.runners import Runner
from google.genai.types import Content, Part
from typing import Optional, Dict, Any
import logging

logger = logging.getLogger(__name__)


class ADKSessionManager:
    """
    Manages ADK sessions for the navigation agent.
    Bridges StateManager with ADK's session system.
    """

    def __init__(self, agent, app_name: str = "blindfolded_navigation"):
        """
        Initialize ADK session manager.

        Args:
            agent: The ADK agent instance
            app_name: Application name for sessions
        """
        self.agent = agent
        self.app_name = app_name
        self.user_id = "blindfolded_user"  # Single user for this application

        # Create ADK session service (in-memory for real-time performance)
        self.session_service = InMemorySessionService()

        # Create ADK runner
        self.runner = Runner(
            agent=self.agent,
            app_name=self.app_name,
            session_service=self.session_service
        )

        # Current session
        self.current_session: Optional[Session] = None
        self.session_id: Optional[str] = None

        logger.info("ADK Session Manager initialized")

    async def start_new_session(self, initial_state: Dict[str, Any]) -> Session:
        """
        Start a new navigation session.

        Args:
            initial_state: Initial state dictionary

        Returns:
            Created session
        """
        try:
            # Create new session with initial state
            self.current_session = await self.session_service.create_session(
                app_name=self.app_name,
                user_id=self.user_id,
                state=initial_state
            )
            self.session_id = self.current_session.id

            logger.info(f"Started new ADK session: {self.session_id}")
            return self.current_session

        except Exception as e:
            logger.error(f"Failed to start session: {e}")
            raise

    async def update_session_state(self, state_updates: Dict[str, Any]):
        """
        Update the current session state.

        Args:
            state_updates: Dictionary of state updates
        """
        if not self.current_session:
            logger.warning("No active session to update")
            return

        try:
            # Get current session
            session = await self.session_service.get_session(
                app_name=self.app_name,
                user_id=self.user_id,
                session_id=self.session_id
            )

            # Update state
            for key, value in state_updates.items():
                session.state[key] = value

            # Note: State updates via runner.run_async will be persisted automatically
            # This method is for reading current state or making manual updates

        except Exception as e:
            logger.error(f"Failed to update session state: {e}")

    async def run_agent(self, query: str = "Generate next navigation instruction") -> str:
        """
        Run the agent to generate a navigation instruction.

        Args:
            query: Query to send to agent

        Returns:
            Generated instruction text
        """
        if not self.current_session:
            logger.error("No active session")
            return "Error: No active session"

        try:
            # Create message content
            message = Content(
                role='user',
                parts=[Part(text=query)]
            )

            # Run agent
            instruction = None
            async for event in self.runner.run_async(
                session_id=self.session_id,
                user_id=self.user_id,
                new_message=message
            ):
                # Check for final response
                if event.is_final_response() and event.content and event.content.parts:
                    instruction = ''.join(part.text or '' for part in event.content.parts)
                    break

                # Log intermediate events (tool calls, etc.)
                if event.content and event.content.parts:
                    for part in event.content.parts:
                        if part.function_call:
                            logger.debug(f"Tool called: {part.function_call.name}")
                        if part.function_response:
                            logger.debug(f"Tool response: {part.function_response.name}")

            return instruction or "No instruction generated"

        except Exception as e:
            logger.error(f"Error running agent: {e}")
            return f"Error: {str(e)}"

    async def get_session_state(self) -> Dict[str, Any]:
        """
        Get current session state.

        Returns:
            Current state dictionary
        """
        if not self.current_session:
            return {}

        try:
            session = await self.session_service.get_session(
                app_name=self.app_name,
                user_id=self.user_id,
                session_id=self.session_id
            )
            return dict(session.state)

        except Exception as e:
            logger.error(f"Failed to get session state: {e}")
            return {}

    async def end_session(self):
        """End the current session and clean up."""
        if not self.current_session:
            return

        try:
            await self.session_service.delete_session(
                app_name=self.app_name,
                user_id=self.user_id,
                session_id=self.session_id
            )

            logger.info(f"Ended session: {self.session_id}")
            self.current_session = None
            self.session_id = None

        except Exception as e:
            logger.error(f"Failed to end session: {e}")

    async def get_session_history(self) -> list:
        """
        Get the event history for the current session.

        Returns:
            List of events
        """
        if not self.current_session:
            return []

        try:
            session = await self.session_service.get_session(
                app_name=self.app_name,
                user_id=self.user_id,
                session_id=self.session_id
            )
            return session.events

        except Exception as e:
            logger.error(f"Failed to get session history: {e}")
            return []
