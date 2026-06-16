"""Grilla de ocupación 2D para planificación de rutas."""
import math
from collections import deque


class OccupancyGrid:
    def __init__(self, bounds, resolution):
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.resolution = resolution
        self.cols = int(math.ceil((self.x_max - self.x_min) / resolution))
        self.rows = int(math.ceil((self.y_max - self.y_min) / resolution))
        self.cells = [[0] * self.cols for _ in range(self.rows)]

    def world_to_cell(self, x, y):
        col = int((x - self.x_min) / self.resolution)
        row = int((y - self.y_min) / self.resolution)
        col = min(max(col, 0), self.cols - 1)
        row = min(max(row, 0), self.rows - 1)
        return col, row

    def cell_to_world(self, col, row):
        x = self.x_min + (col + 0.5) * self.resolution
        y = self.y_min + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, col, row):
        return 0 <= col < self.cols and 0 <= row < self.rows

    def is_free(self, col, row):
        return self.in_bounds(col, row) and self.cells[row][col] == 0

    def mark_obstacle(self, cx, cy, half_x, half_y, inflate):
        x0, x1 = cx - half_x - inflate, cx + half_x + inflate
        y0, y1 = cy - half_y - inflate, cy + half_y + inflate

        col0, row0 = self.world_to_cell(x0, y0)
        col1, row1 = self.world_to_cell(x1, y1)

        for row in range(min(row0, row1), max(row0, row1) + 1):
            for col in range(min(col0, col1), max(col0, col1) + 1):
                px, py = self.cell_to_world(col, row)
                dx = max(abs(px - cx) - half_x, 0.0)
                dy = max(abs(py - cy) - half_y, 0.0)
                if math.hypot(dx, dy) <= inflate:
                    self.cells[row][col] = 1

    @classmethod
    def from_scenario(cls, scenario, resolution, inflate):
        grid = cls(scenario['bounds'], resolution)
        for cx, cy, half_x, half_y in scenario['obstacles']:
            grid.mark_obstacle(cx, cy, half_x, half_y, inflate)
        return grid

    def clearance_grid(self):
        INF = float('inf')
        dist = [[INF] * self.cols for _ in range(self.rows)]
        q = deque()
        for row in range(self.rows):
            for col in range(self.cols):
                if self.cells[row][col] == 1:
                    dist[row][col] = 0.0
                    q.append((col, row))

        while q:
            col, row = q.popleft()
            d = dist[row][col]
            for dx, dy, step in (
                (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
            ):
                ncol, nrow = col + dx, row + dy
                if self.in_bounds(ncol, nrow) and dist[nrow][ncol] > d + step:
                    dist[nrow][ncol] = d + step
                    q.append((ncol, nrow))

        return dist
