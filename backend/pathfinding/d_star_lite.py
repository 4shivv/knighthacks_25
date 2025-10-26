"""
D* Lite incremental pathfinding algorithm for ELEGOO Smart Robot Car.
Adapted from: https://github.com/AtDinesh/Dstar-lite-pathplanner

D* Lite is an incremental heuristic search algorithm that efficiently
replans shortest paths when edge costs change (new obstacles discovered).
"""

import numpy as np
import logging
from typing import Tuple, List, Optional
from .priority_queue import PriorityQueue, Priority
from .occupancy_grid import OccupancyGrid, OBSTACLE, UNOCCUPIED
from .utils import heuristic, Vertices, Vertex

logger = logging.getLogger(__name__)


class DStarLite:
    """
    D* Lite incremental shortest path algorithm.

    Efficiently replans paths when obstacles are discovered.
    """

    def __init__(self, occupancy_grid: OccupancyGrid, s_start: Tuple[int, int], s_goal: Tuple[int, int]):
        """
        Initialize D* Lite pathfinder.

        Args:
            occupancy_grid: The occupancy grid map
            s_start: Start position (x, y) in grid coordinates
            s_goal: Goal position (x, y) in grid coordinates
        """
        self.map = occupancy_grid
        self.s_start = s_start
        self.s_goal = s_goal
        self.s_last = s_start
        self.k_m = 0  # Accumulation for path cost updates

        # Priority queue for vertices to be processed
        self.U = PriorityQueue()

        # rhs[s] = one-step lookahead value
        # g[s] = cost estimate
        self.rhs = np.ones((occupancy_grid.x_dim, occupancy_grid.y_dim)) * np.inf
        self.g = self.rhs.copy()

        # Initialize goal
        self.rhs[self.s_goal] = 0
        self.U.insert(self.s_goal, Priority(heuristic(self.s_start, self.s_goal), 0))

        logger.info(f"D* Lite initialized: start={s_start}, goal={s_goal}")

    def calculate_key(self, s: Tuple[int, int]) -> Priority:
        """
        Calculate priority key for a vertex.

        Args:
            s: Vertex position (x, y)

        Returns:
            Priority with two key values for lexicographic ordering
        """
        k1 = min(self.g[s], self.rhs[s]) + heuristic(self.s_start, s) + self.k_m
        k2 = min(self.g[s], self.rhs[s])
        return Priority(k1, k2)

    def c(self, u: Tuple[int, int], v: Tuple[int, int]) -> float:
        """
        Calculate cost between two adjacent vertices.

        Args:
            u: From vertex (x, y)
            v: To vertex (x, y)

        Returns:
            Euclidean distance if path is free, infinity if obstacle
        """
        if not self.map.is_unoccupied(u) or not self.map.is_unoccupied(v):
            return float('inf')
        else:
            return heuristic(u, v)

    def contain(self, u: Tuple[int, int]) -> bool:
        """Check if vertex is in priority queue."""
        return u in self.U.vertices_in_heap

    def update_vertex(self, u: Tuple[int, int]):
        """
        Update vertex in priority queue based on g and rhs values.

        Args:
            u: Vertex position (x, y)
        """
        if self.g[u] != self.rhs[u] and self.contain(u):
            self.U.update(u, self.calculate_key(u))
        elif self.g[u] != self.rhs[u] and not self.contain(u):
            self.U.insert(u, self.calculate_key(u))
        elif self.g[u] == self.rhs[u] and self.contain(u):
            self.U.remove(u)

    def compute_shortest_path(self):
        """
        Compute shortest path from start to goal using D* Lite algorithm.
        Updates g and rhs values for vertices.
        """
        while self.U.top_key() < self.calculate_key(self.s_start) or self.rhs[self.s_start] > self.g[self.s_start]:
            u = self.U.top()
            k_old = self.U.top_key()
            k_new = self.calculate_key(u)

            if k_old < k_new:
                self.U.update(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.U.remove(u)
                pred = self.map.succ(vertex=u)
                for s in pred:
                    if s != self.s_goal:
                        self.rhs[s] = min(self.rhs[s], self.c(s, u) + self.g[u])
                    self.update_vertex(s)
            else:
                g_old = self.g[u]
                self.g[u] = float('inf')
                pred = self.map.succ(vertex=u)
                pred.append(u)
                for s in pred:
                    if self.rhs[s] == self.c(s, u) + g_old:
                        if s != self.s_goal:
                            min_s = float('inf')
                            succ = self.map.succ(vertex=s)
                            for s_ in succ:
                                temp = self.c(s, s_) + self.g[s_]
                                if min_s > temp:
                                    min_s = temp
                            self.rhs[s] = min_s
                    self.update_vertex(s)

    def get_next_waypoint(self, current_position: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        """
        Get the next waypoint to move to from current position.

        Args:
            current_position: Current robot position in grid (x, y)

        Returns:
            Next waypoint position (x, y) or None if at goal or no path
        """
        if current_position == self.s_goal:
            return None

        # Check if path exists
        if self.rhs[current_position] == float('inf'):
            logger.warning(f"No known path from {current_position} to {self.s_goal}")
            return None

        # Find best successor
        succ = self.map.succ(current_position, avoid_obstacles=False)
        min_cost = float('inf')
        best_successor = None

        for s in succ:
            cost = self.c(current_position, s) + self.g[s]
            if cost < min_cost:
                min_cost = cost
                best_successor = s

        return best_successor

    def get_full_path(self, start_position: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Get the full path from start to goal.

        Args:
            start_position: Starting position (x, y)

        Returns:
            List of waypoints from start to goal
        """
        path = [start_position]
        current = start_position

        # Limit path length to prevent infinite loops
        max_steps = self.map.x_dim * self.map.y_dim

        for _ in range(max_steps):
            next_wp = self.get_next_waypoint(current)
            if next_wp is None:
                # Reached goal or no path
                break

            path.append(next_wp)
            current = next_wp

            if current == self.s_goal:
                break

        return path

    def update_start(self, new_start: Tuple[int, int]):
        """
        Update the start position (robot moved).

        Args:
            new_start: New start position (x, y)
        """
        self.s_start = new_start

    def update_goal(self, new_goal: Tuple[int, int]):
        """
        Update the goal position.

        Args:
            new_goal: New goal position (x, y)
        """
        old_goal = self.s_goal
        self.s_goal = new_goal

        # Reset goal values
        self.rhs[old_goal] = float('inf')
        self.rhs[new_goal] = 0

        # Recompute path
        self.k_m = 0
        self.U = PriorityQueue()
        self.U.insert(new_goal, Priority(heuristic(self.s_start, new_goal), 0))

        logger.info(f"Goal updated: {old_goal} -> {new_goal}")

    def update_map(self, changed_cells: List[Tuple[int, int]]):
        """
        Update D* Lite after map changes (obstacles added/removed).

        Args:
            changed_cells: List of (x, y) grid positions that changed
        """
        if not changed_cells:
            return

        # Update k_m
        self.k_m += heuristic(self.s_last, self.s_start)
        self.s_last = self.s_start

        # For each changed cell, update affected edges
        for pos in changed_cells:
            # Get neighbors
            neighbors = self.map.succ(pos, avoid_obstacles=False)

            # Update rhs values for neighbors
            for neighbor in neighbors:
                if neighbor != self.s_goal:
                    # Recalculate rhs
                    min_cost = float('inf')
                    succ = self.map.succ(neighbor, avoid_obstacles=False)
                    for s in succ:
                        cost = self.c(neighbor, s) + self.g[s]
                        if cost < min_cost:
                            min_cost = cost
                    self.rhs[neighbor] = min_cost

                self.update_vertex(neighbor)

        # Recompute shortest path
        self.compute_shortest_path()

        logger.debug(f"Map updated: {len(changed_cells)} cells changed")

    def replan(self, new_start: Tuple[int, int], changed_cells: Optional[List[Tuple[int, int]]] = None):
        """
        Replan path from new start position, optionally with map changes.

        Args:
            new_start: New start position (x, y)
            changed_cells: Optional list of changed grid cells
        """
        self.update_start(new_start)

        if changed_cells:
            self.update_map(changed_cells)
        else:
            self.compute_shortest_path()
