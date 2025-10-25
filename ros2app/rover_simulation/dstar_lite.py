
import heapq
import math
from typing import Tuple, List, Set, Dict, Optional


class DStarLite:
    """D* Lite pathfinding algorithm for dynamic environments."""

    def __init__(self, grid_size: Tuple[int, int], start: Tuple[int, int],
                 goal: Tuple[int, int]):
        """Initialize D* Lite planner.
        
        Args:
            grid_size: (width, height) of the grid
            start: Starting position (x, y)
            goal: Goal position (x, y)
        """
        self.grid_width, self.grid_height = grid_size
        self.start = start
        self.goal = goal
        self.km = 0
        
        self.rhs: Dict[Tuple[int, int], float] = {}
        self.g: Dict[Tuple[int, int], float] = {}
        self.obstacles: Set[Tuple[int, int]] = set()
        self.open_list: List[Tuple[Tuple[float, float], Tuple[int, int]]] = []
        
        self._initialize()

    def _initialize(self) -> None:
        """Initialize the planner state."""
        self.rhs[self.goal] = 0
        heapq.heappush(self.open_list,
                      (self._calculate_key(self.goal), self.goal))

    def _calculate_key(self, s: Tuple[int, int]) -> Tuple[float, float]:
        """Calculate priority key for state s.
        
        Args:
            s: State position (x, y)
            
        Returns:
            Priority key tuple (k1, k2)
        """
        g_rhs = min(self.g.get(s, float('inf')), self.rhs.get(s, float('inf')))
        return (g_rhs + self._heuristic(self.start, s) + self.km,
                g_rhs)

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic.
        
        Args:
            a: First position
            b: Second position
            
        Returns:
            Euclidean distance
        """
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def _get_neighbors(self, s: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighbors of state s.
        
        Args:
            s: Current position
            
        Returns:
            List of neighbor positions
        """
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = s[0] + dx, s[1] + dy
            if (0 <= nx < self.grid_width and 0 <= ny < self.grid_height):
                neighbors.append((nx, ny))
        return neighbors

    def _cost(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate cost between adjacent cells.
        
        Args:
            a: From position
            b: To position
            
        Returns:
            Movement cost (inf if obstacle)
        """
        if b in self.obstacles:
            return float('inf')
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return math.sqrt(dx**2 + dy**2)

    def _update_vertex(self, u: Tuple[int, int]) -> None:
        """Update vertex u in the priority queue.
        
        Args:
            u: Position to update
        """
        if u != self.goal:
            min_cost = float('inf')
            for s in self._get_neighbors(u):
                cost = self._cost(u, s) + self.g.get(s, float('inf'))
                min_cost = min(min_cost, cost)
            self.rhs[u] = min_cost

        # Remove u from open list if present
        self.open_list = [(k, s) for k, s in self.open_list if s != u]
        heapq.heapify(self.open_list)

        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            heapq.heappush(self.open_list,
                          (self._calculate_key(u), u))

    def _compute_shortest_path(self) -> None:
        """Compute shortest path from start to goal."""
        while (self.open_list and
               (self.open_list[0][0] < self._calculate_key(self.start) or
                self.rhs.get(self.start, float('inf')) !=
                self.g.get(self.start, float('inf')))):
            
            k_old, u = heapq.heappop(self.open_list)
            k_new = self._calculate_key(u)

            if k_old < k_new:
                heapq.heappush(self.open_list, (k_new, u))
            elif self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                for s in self._get_neighbors(u):
                    self._update_vertex(s)
            else:
                self.g[u] = float('inf')
                self._update_vertex(u)
                for s in self._get_neighbors(u):
                    self._update_vertex(s)

    def update_obstacles(self, new_obstacles: Set[Tuple[int, int]]) -> None:
        """Update obstacle map and replan if necessary.
        
        Args:
            new_obstacles: Set of obstacle positions
        """
        changed_cells = new_obstacles.symmetric_difference(self.obstacles)
        
        if not changed_cells:
            return

        self.km += self._heuristic(self.start, self.goal)
        
        for cell in changed_cells:
            if cell in new_obstacles:
                self.obstacles.add(cell)
            else:
                self.obstacles.discard(cell)
            
            for neighbor in self._get_neighbors(cell):
                self._update_vertex(neighbor)
            self._update_vertex(cell)

        self.obstacles = new_obstacles.copy()

    def get_path(self) -> Optional[List[Tuple[int, int]]]:
        """Get the current best path from start to goal.
        
        Returns:
            List of positions forming the path, or None if no path exists
        """
        self._compute_shortest_path()
        
        if self.g.get(self.start, float('inf')) == float('inf'):
            return None

        path = [self.start]
        current = self.start

        while current != self.goal:
            min_cost = float('inf')
            next_state = None

            for neighbor in self._get_neighbors(current):
                cost = (self._cost(current, neighbor) +
                       self.g.get(neighbor, float('inf')))
                if cost < min_cost:
                    min_cost = cost
                    next_state = neighbor

            if next_state is None:
                return None

            path.append(next_state)
            current = next_state

            if len(path) > self.grid_width * self.grid_height:
                return None

        return path

    def move_start(self, new_start: Tuple[int, int]) -> None:
        """Update start position (robot moved).
        
        Args:
            new_start: New starting position
        """
        self.start = new_start