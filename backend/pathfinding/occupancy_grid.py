"""
Occupancy Grid Map for ELEGOO Smart Robot Car pathfinding.
Adapted from original D* Lite implementation.
"""

import numpy as np
import logging
from typing import Dict, List, Tuple
from .utils import get_movements_4n, get_movements_8n, heuristic, Vertices, Vertex

logger = logging.getLogger(__name__)

OBSTACLE = 255
UNOCCUPIED = 0


class OccupancyGrid:
    """
    2D occupancy grid map for pathfinding.
    Stores obstacle information and provides neighbor queries.
    """

    def __init__(self, x_dim: int, y_dim: int, exploration_setting: str = '8N'):
        """
        Initialize occupancy grid map.

        Args:
            x_dim: Grid dimension in x direction (rows)
            y_dim: Grid dimension in y direction (columns)
            exploration_setting: '4N' for 4-connectivity, '8N' for 8-connectivity
        """
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.map_extents = (x_dim, y_dim)

        # Initialize empty grid (all cells unoccupied)
        self.occupancy_grid_map = np.zeros(self.map_extents, dtype=np.uint8)

        self.exploration_setting = exploration_setting
        logger.info(f"OccupancyGrid initialized: {x_dim}x{y_dim} cells, {exploration_setting} connectivity")

    def get_map(self) -> np.ndarray:
        """Get the current occupancy grid map."""
        return self.occupancy_grid_map

    def set_map(self, new_ogrid: np.ndarray):
        """Set the entire occupancy grid map."""
        self.occupancy_grid_map = new_ogrid

    def is_unoccupied(self, pos: Tuple[int, int]) -> bool:
        """
        Check if a cell is unoccupied (free).

        Args:
            pos: (x, y) grid position

        Returns:
            True if cell is unoccupied, False if obstacle
        """
        (x, y) = (round(pos[0]), round(pos[1]))
        (row, col) = (x, y)

        if not self.in_bounds((x, y)):
            return False

        return self.occupancy_grid_map[row][col] == UNOCCUPIED

    def in_bounds(self, cell: Tuple[int, int]) -> bool:
        """
        Check if coordinates are within grid bounds.

        Args:
            cell: (x, y) grid position

        Returns:
            True if within bounds, False otherwise
        """
        (x, y) = cell
        return 0 <= x < self.x_dim and 0 <= y < self.y_dim

    def filter(self, neighbors: List[Tuple[int, int]], avoid_obstacles: bool) -> List[Tuple[int, int]]:
        """
        Filter neighbors to only include valid cells.

        Args:
            neighbors: List of potential neighbor positions
            avoid_obstacles: If True, filter out obstacle cells

        Returns:
            Filtered list of valid neighbors
        """
        if avoid_obstacles:
            return [node for node in neighbors if self.in_bounds(node) and self.is_unoccupied(node)]
        return [node for node in neighbors if self.in_bounds(node)]

    def succ(self, vertex: Tuple[int, int], avoid_obstacles: bool = False) -> List[Tuple[int, int]]:
        """
        Get successors (neighbors) of a vertex.

        Args:
            vertex: (x, y) grid position
            avoid_obstacles: If True, exclude obstacle neighbors

        Returns:
            List of successor positions
        """
        (x, y) = vertex

        if self.exploration_setting == '4N':
            movements = get_movements_4n(x=x, y=y)
        else:
            movements = get_movements_8n(x=x, y=y)

        # Reverse for aesthetic path (makes path smoother)
        if (x + y) % 2 == 0:
            movements.reverse()

        filtered_movements = self.filter(neighbors=movements, avoid_obstacles=avoid_obstacles)
        return list(filtered_movements)

    def set_obstacle(self, pos: Tuple[int, int]):
        """
        Mark a cell as an obstacle.

        Args:
            pos: (x, y) grid position
        """
        (x, y) = (round(pos[0]), round(pos[1]))
        (row, col) = (x, y)

        if self.in_bounds((x, y)):
            self.occupancy_grid_map[row, col] = OBSTACLE

    def remove_obstacle(self, pos: Tuple[int, int]):
        """
        Mark an obstacle cell as unoccupied.

        Args:
            pos: (x, y) grid position
        """
        (x, y) = (round(pos[0]), round(pos[1]))
        (row, col) = (x, y)

        if self.in_bounds((x, y)):
            self.occupancy_grid_map[row, col] = UNOCCUPIED

    def local_observation(self, global_position: Tuple[int, int], view_range: int = 2) -> Dict[Tuple[int, int], int]:
        """
        Get local observation around a position (for sensor updates).

        Args:
            global_position: (x, y) robot position in grid
            view_range: How many cells ahead to look

        Returns:
            Dictionary mapping positions to occupancy values
        """
        (px, py) = global_position
        nodes = [
            (x, y)
            for x in range(px - view_range, px + view_range + 1)
            for y in range(py - view_range, py + view_range + 1)
            if self.in_bounds((x, y))
        ]
        return {
            node: UNOCCUPIED if self.is_unoccupied(pos=node) else OBSTACLE
            for node in nodes
        }

    def update_from_sensor_data(self, occupancy_data: np.ndarray):
        """
        Update grid with new occupancy data from sensors.

        Args:
            occupancy_data: Numpy array with same dimensions as grid
        """
        if occupancy_data.shape != self.occupancy_grid_map.shape:
            logger.error(
                f"Occupancy data shape {occupancy_data.shape} doesn't match "
                f"grid shape {self.occupancy_grid_map.shape}"
            )
            return

        self.occupancy_grid_map = occupancy_data.astype(np.uint8)
        logger.debug("Occupancy grid updated from sensor data")

    def get_obstacle_count(self) -> int:
        """Get the number of obstacle cells in the grid."""
        return np.sum(self.occupancy_grid_map == OBSTACLE)

    def get_unoccupied_count(self) -> int:
        """Get the number of unoccupied cells in the grid."""
        return np.sum(self.occupancy_grid_map == UNOCCUPIED)
