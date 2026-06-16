"""Seguimiento de ruta: control proporcional de rumbo hacia cada waypoint."""
import math

WAYPOINT_TOLERANCE = 0.06   # [m] distancia para considerar alcanzado un waypoint intermedio
GOAL_TOLERANCE = 0.20       # [m] distancia a la meta real para darse por llegado
K_HEADING = 1.0              # ganancia proporcional de giro [rad/s por rad de error]
CRUISE_SPEED = 0.08          # [m/s] velocidad lineal de crucero (e-puck max ~0.1287 m/s)


class PathFollower:
    def __init__(self, waypoints, goal=None):
        self.waypoints = list(waypoints)
        self.goal = goal if goal is not None else (waypoints[-1] if waypoints else None)
        self.index = 0
        self.done = len(self.waypoints) == 0

    def step(self, x, y, theta):
        """Retorna (v, omega) deseados dada la pose actual estimada por odometría."""
        if self.done:
            return 0.0, 0.0

        if math.hypot(self.goal[0] - x, self.goal[1] - y) < GOAL_TOLERANCE:
            self.done = True
            return 0.0, 0.0

        target_x, target_y = self.waypoints[self.index]
        distance = math.hypot(target_x - x, target_y - y)

        if distance < WAYPOINT_TOLERANCE:
            self.index += 1
            if self.index >= len(self.waypoints):
                self.done = True
                return 0.0, 0.0
            target_x, target_y = self.waypoints[self.index]
            distance = math.hypot(target_x - x, target_y - y)

        target_heading = math.atan2(target_y - y, target_x - x)
        heading_error = math.atan2(
            math.sin(target_heading - theta), math.cos(target_heading - theta)
        )

        omega = K_HEADING * heading_error
        v = CRUISE_SPEED * max(0.0, math.cos(heading_error))

        return v, omega
