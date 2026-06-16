"""Planificación de rutas con A* sobre una grilla de ocupación."""
import heapq
import math

_NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def _heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

CLEARANCE_TARGET = 3.0
CLEARANCE_WEIGHT = 0.6


def astar(grid, start_world, goal_world):
    start = grid.world_to_cell(*start_world)
    goal = grid.world_to_cell(*goal_world)

    if not grid.is_free(*start):
        raise ValueError("La posición inicial cae sobre una celda ocupada")
    if not grid.is_free(*goal):
        raise ValueError("La meta cae sobre una celda ocupada")

    clearance = grid.clearance_grid()

    open_heap = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    visited = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return _reconstruct_path(grid, came_from, current)

        for dx, dy, step_cost in _NEIGHBORS:
            neighbor = (current[0] + dx, current[1] + dy)
            if not grid.is_free(*neighbor):
                continue
            deficit = max(0.0, CLEARANCE_TARGET - clearance[neighbor[1]][neighbor[0]])
            penalty = CLEARANCE_WEIGHT * deficit * deficit
            tentative_g = g_score[current] + step_cost + penalty
            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + _heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return None


def _reconstruct_path(grid, came_from, current):
    cells = [current]
    while current in came_from:
        current = came_from[current]
        cells.append(current)
    cells.reverse()

    waypoints = [grid.cell_to_world(c, r) for c, r in cells]
    return _simplify(waypoints)


def _simplify(waypoints, tol=1e-6):
    if len(waypoints) <= 2:
        return waypoints

    simplified = [waypoints[0]]
    for i in range(1, len(waypoints) - 1):
        x0, y0 = simplified[-1]
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        dir1 = (x1 - x0, y1 - y0)
        dir2 = (x2 - x1, y2 - y1)
        cross = dir1[0] * dir2[1] - dir1[1] * dir2[0]
        if abs(cross) > tol:
            simplified.append(waypoints[i])
    simplified.append(waypoints[-1])
    return simplified


def path_length(waypoints):
    return sum(
        math.hypot(waypoints[i + 1][0] - waypoints[i][0], waypoints[i + 1][1] - waypoints[i][1])
        for i in range(len(waypoints) - 1)
    )
