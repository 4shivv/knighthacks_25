"""
Path Planning Loop for ELEGOO Smart Robot Car V4.0
Uses D* Lite algorithm for incremental pathfinding with obstacle updates.

Runs at 1 Hz (every 1 second)
"""

import asyncio
import logging
import time
from typing import Optional, List, Tuple

from pathfinding.d_star_lite import DStarLite
from pathfinding.occupancy_grid import OccupancyGrid
from pathfinding.utils import world_to_grid, grid_to_world

logger = logging.getLogger(__name__)


class PathPlanningLoop:
    """
    Path planning loop that uses D* Lite for incremental pathfinding.
    Updates path when obstacles are discovered or goal changes.
    """

    def __init__(
        self,
        state_manager,
        odometry,
        grid_size_x: int = 100,  # 10m x 0.1m resolution
        grid_size_y: int = 100,
        grid_resolution: float = 0.1,  # meters per cell
        interval_ms: int = 1000  # 1 Hz
    ):
        """
        Initialize path planning loop.

        Args:
            state_manager: StateManager instance
            odometry: Odometry instance
            grid_size_x: Grid size in X direction (cells)
            grid_size_y: Grid size in Y direction (cells)
            grid_resolution: Size of each grid cell in meters
            interval_ms: Loop interval in milliseconds
        """
        self.state_manager = state_manager
        self.odometry = odometry
        self.interval_seconds = interval_ms / 1000.0
        self.running = False

        # Grid parameters
        self.grid_resolution = grid_resolution
        self.grid_size_x = grid_size_x
        self.grid_size_y = grid_size_y

        # Create occupancy grid
        self.occupancy_grid = OccupancyGrid(
            x_dim=grid_size_x,
            y_dim=grid_size_y,
            exploration_setting='8N'  # 8-connectivity for smoother paths
        )

        # D* Lite pathfinder (will be initialized when goal is set)
        self.dstar: Optional[DStarLite] = None
        self.current_path: List[Tuple[float, float]] = []  # World coordinates

        # Statistics
        self.total_replans = 0
        self.total_obstacles_detected = 0
        self.last_replan_time = 0

        logger.info(
            f"Path planning loop initialized: "
            f"{grid_size_x}x{grid_size_y} grid, {grid_resolution}m resolution, "
            f"{interval_ms}ms interval"
        )

    async def run(self):
        """Main path planning loop - runs continuously."""
        self.running = True
        logger.info("Path planning loop started")

        while self.running:
            try:
                # Check if we have a goal
                target_object = self.state_manager.target_object
                if not target_object:
                    # No goal set yet
                    await asyncio.sleep(self.interval_seconds)
                    continue

                # Get current robot position from odometry
                current_pos = self.odometry.get_position_tuple()  # (x, y) in meters
                current_grid = world_to_grid(current_pos[0], current_pos[1], self.grid_resolution)

                # Get goal position from target object
                goal_pos = (target_object.position.x, target_object.position.y)  # (x, y) in meters
                goal_grid = world_to_grid(goal_pos[0], goal_pos[1], self.grid_resolution)

                # Check if goal is within grid bounds
                if not self.occupancy_grid.in_bounds(goal_grid):
                    logger.warning(f"Goal {goal_grid} is outside grid bounds")
                    await asyncio.sleep(self.interval_seconds)
                    continue

                # Initialize D* Lite if not already done or goal changed
                if self.dstar is None or self.dstar.s_goal != goal_grid:
                    logger.info(f"Initializing D* Lite: start={current_grid}, goal={goal_grid}")
                    self.dstar = DStarLite(
                        occupancy_grid=self.occupancy_grid,
                        s_start=current_grid,
                        s_goal=goal_grid
                    )
                    # Compute initial path
                    self.dstar.compute_shortest_path()
                    self.total_replans += 1

                # Update occupancy grid from sensor fusion
                # (This will be populated by sensor fusion loop in the future)
                changed_cells = await self._update_occupancy_grid()

                # If obstacles changed, replan
                if changed_cells:
                    self.total_obstacles_detected += len(changed_cells)
                    logger.info(f"Replanning due to {len(changed_cells)} map changes")
                    self.dstar.replan(current_grid, changed_cells)
                    self.total_replans += 1
                    self.last_replan_time = time.time()

                # Get next waypoint from D* Lite
                next_grid_waypoint = self.dstar.get_next_waypoint(current_grid)

                if next_grid_waypoint:
                    # Convert to world coordinates
                    next_world_waypoint = grid_to_world(
                        next_grid_waypoint[0],
                        next_grid_waypoint[1],
                        self.grid_resolution
                    )

                    # Update state manager with next waypoint
                    self.state_manager.update_next_waypoint(next_world_waypoint)

                    logger.debug(
                        f"Next waypoint: grid={next_grid_waypoint}, "
                        f"world={next_world_waypoint}"
                    )

                    # Get full path for visualization
                    full_path_grid = self.dstar.get_full_path(current_grid)
                    self.current_path = [
                        grid_to_world(gx, gy, self.grid_resolution)
                        for (gx, gy) in full_path_grid
                    ]
                else:
                    # Reached goal or no path
                    if current_grid == goal_grid:
                        logger.info("Reached goal!")
                        self.state_manager.update_next_waypoint(None)
                    else:
                        logger.warning("No path to goal!")

                # Wait for next cycle
                await asyncio.sleep(self.interval_seconds)

            except Exception as e:
                logger.error(f"Error in path planning loop: {e}", exc_info=True)
                await asyncio.sleep(self.interval_seconds)

        logger.info("Path planning loop stopped")

    async def _update_occupancy_grid(self) -> List[Tuple[int, int]]:
        """
        Update occupancy grid from sensor fusion data.

        Returns:
            List of changed grid cells (x, y)
        """
        # Get occupancy grid from state manager
        # (This will be populated by sensor fusion loop)
        occupancy_data = self.state_manager.occupancy_grid

        if occupancy_data is None:
            return []

        # Compare with previous grid to find changes
        previous_grid = self.occupancy_grid.get_map().copy()
        self.occupancy_grid.update_from_sensor_data(occupancy_data)
        new_grid = self.occupancy_grid.get_map()

        # Find changed cells
        changed_mask = (previous_grid != new_grid)
        changed_indices = list(zip(*changed_mask.nonzero()))

        return changed_indices

    def get_stats(self) -> dict:
        """Get path planning loop statistics."""
        return {
            "running": self.running,
            "total_replans": self.total_replans,
            "total_obstacles_detected": self.total_obstacles_detected,
            "last_replan_time": self.last_replan_time,
            "has_goal": self.dstar is not None,
            "current_path_length": len(self.current_path),
            "grid_size": f"{self.grid_size_x}x{self.grid_size_y}",
            "grid_resolution": f"{self.grid_resolution}m",
            "obstacle_count": self.occupancy_grid.get_obstacle_count()
        }

    def reset(self):
        """Reset path planner (clear path and D* Lite instance)."""
        self.dstar = None
        self.current_path = []
        logger.info("Path planner reset")
