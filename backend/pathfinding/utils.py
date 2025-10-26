"""
Utility functions for D* Lite pathfinding.
Adapted from original D* Lite implementation.
"""

import math
from typing import List, Tuple


class Vertex:
    """Represents a vertex in the pathfinding graph with edges and costs."""

    def __init__(self, pos: Tuple[int, int]):
        self.pos = pos
        self.edges_and_costs = {}

    def add_edge_with_cost(self, succ: Tuple[int, int], cost: float):
        """Add an edge to a successor vertex with a cost."""
        if succ != self.pos:
            self.edges_and_costs[succ] = cost

    @property
    def edges_and_c_old(self):
        """Get edges and their old costs."""
        return self.edges_and_costs


class Vertices:
    """Collection of vertices."""

    def __init__(self):
        self.list = []

    def add_vertex(self, v: Vertex):
        """Add a vertex to the collection."""
        self.list.append(v)

    @property
    def vertices(self):
        """Get all vertices."""
        return self.list


def heuristic(p: Tuple[int, int], q: Tuple[int, int]) -> float:
    """
    Compute Euclidean distance heuristic between two grid points.

    Args:
        p: (x, y) grid coordinate
        q: (x, y) grid coordinate

    Returns:
        Euclidean distance between points
    """
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)


def get_movements_4n(x: int, y: int) -> List[Tuple[int, int]]:
    """
    Get all possible 4-connectivity movements (up, down, left, right).

    Args:
        x: Current x position
        y: Current y position

    Returns:
        List of (x, y) positions for 4-connected neighbors
    """
    return [
        (x + 1, y + 0),
        (x + 0, y + 1),
        (x - 1, y + 0),
        (x + 0, y - 1)
    ]


def get_movements_8n(x: int, y: int) -> List[Tuple[int, int]]:
    """
    Get all possible 8-connectivity movements (including diagonals).

    Args:
        x: Current x position
        y: Current y position

    Returns:
        List of (x, y) positions for 8-connected neighbors
    """
    return [
        (x + 1, y + 0),
        (x + 0, y + 1),
        (x - 1, y + 0),
        (x + 0, y - 1),
        (x + 1, y + 1),
        (x - 1, y + 1),
        (x - 1, y - 1),
        (x + 1, y - 1)
    ]


def world_to_grid(x: float, y: float, grid_resolution: float) -> Tuple[int, int]:
    """
    Convert world coordinates (meters) to grid coordinates.

    Args:
        x: World X coordinate in meters
        y: World Y coordinate in meters
        grid_resolution: Size of each grid cell in meters

    Returns:
        (grid_x, grid_y) tuple
    """
    grid_x = int(round(x / grid_resolution))
    grid_y = int(round(y / grid_resolution))
    return (grid_x, grid_y)


def grid_to_world(grid_x: int, grid_y: int, grid_resolution: float) -> Tuple[float, float]:
    """
    Convert grid coordinates to world coordinates (meters).

    Args:
        grid_x: Grid X coordinate
        grid_y: Grid Y coordinate
        grid_resolution: Size of each grid cell in meters

    Returns:
        (world_x, world_y) tuple in meters
    """
    world_x = grid_x * grid_resolution
    world_y = grid_y * grid_resolution
    return (world_x, world_y)
