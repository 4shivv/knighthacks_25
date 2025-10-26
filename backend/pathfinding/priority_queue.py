"""
Priority queue implementation for D* Lite pathfinding.
Adapted from original D* Lite implementation.
"""

from typing import Tuple


class Priority:
    """Handle lexicographic order of keys for D* Lite."""

    def __init__(self, k1: float, k2: float):
        """
        Initialize priority with two key values.

        Args:
            k1: First key value
            k2: Second key value
        """
        self.k1 = k1
        self.k2 = k2

    def __lt__(self, other):
        """Lexicographic 'lower than' comparison."""
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 < other.k2)

    def __le__(self, other):
        """Lexicographic 'lower than or equal' comparison."""
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 <= other.k2)


class PriorityNode:
    """Handle lexicographic order of vertices in priority queue."""

    def __init__(self, priority: Priority, vertex: Tuple[int, int]):
        """
        Initialize priority node.

        Args:
            priority: Priority of the vertex
            vertex: (x, y) grid position
        """
        self.priority = priority
        self.vertex = vertex

    def __le__(self, other):
        """'Lower than or equal' comparison based on priority."""
        return self.priority <= other.priority

    def __lt__(self, other):
        """'Lower than' comparison based on priority."""
        return self.priority < other.priority


class PriorityQueue:
    """Min-heap priority queue for D* Lite algorithm."""

    def __init__(self):
        self.heap = []
        self.vertices_in_heap = []

    def top(self) -> Tuple[int, int]:
        """Get the vertex with minimum priority without removing it."""
        return self.heap[0].vertex

    def top_key(self) -> Priority:
        """Get the minimum priority without removing it."""
        if len(self.heap) == 0:
            return Priority(float('inf'), float('inf'))
        return self.heap[0].priority

    def pop(self) -> PriorityNode:
        """Pop the smallest item off the heap, maintaining the heap invariant."""
        lastelt = self.heap.pop()
        self.vertices_in_heap.remove(lastelt.vertex)
        if self.heap:
            returnitem = self.heap[0]
            self.heap[0] = lastelt
            self._siftup(0)
        else:
            returnitem = lastelt
        return returnitem

    def insert(self, vertex: Tuple[int, int], priority: Priority):
        """Insert vertex with priority into heap."""
        item = PriorityNode(priority, vertex)
        self.vertices_in_heap.append(vertex)
        self.heap.append(item)
        self._siftdown(0, len(self.heap) - 1)

    def remove(self, vertex: Tuple[int, int]):
        """Remove a vertex from the heap."""
        self.vertices_in_heap.remove(vertex)
        for index, priority_node in enumerate(self.heap):
            if priority_node.vertex == vertex:
                self.heap[index] = self.heap[len(self.heap) - 1]
                self.heap.remove(self.heap[len(self.heap) - 1])
                break
        self.build_heap()

    def update(self, vertex: Tuple[int, int], priority: Priority):
        """Update the priority of a vertex in the heap."""
        for index, priority_node in enumerate(self.heap):
            if priority_node.vertex == vertex:
                self.heap[index].priority = priority
                break
        self.build_heap()

    def build_heap(self):
        """Transform list into a heap, in-place, in O(len(x)) time."""
        n = len(self.heap)
        for i in reversed(range(n // 2)):
            self._siftup(i)

    def _siftdown(self, startpos: int, pos: int):
        """Restore heap invariant by sifting down."""
        newitem = self.heap[pos]
        while pos > startpos:
            parentpos = (pos - 1) >> 1
            parent = self.heap[parentpos]
            if newitem < parent:
                self.heap[pos] = parent
                pos = parentpos
                continue
            break
        self.heap[pos] = newitem

    def _siftup(self, pos: int):
        """Restore heap invariant by sifting up."""
        endpos = len(self.heap)
        startpos = pos
        newitem = self.heap[pos]
        childpos = 2 * pos + 1
        while childpos < endpos:
            rightpos = childpos + 1
            if rightpos < endpos and not self.heap[childpos] < self.heap[rightpos]:
                childpos = rightpos
            self.heap[pos] = self.heap[childpos]
            pos = childpos
            childpos = 2 * pos + 1
        self.heap[pos] = newitem
        self._siftdown(startpos, pos)
