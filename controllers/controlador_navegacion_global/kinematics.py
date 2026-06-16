import math
L = 0.052     # distancia entre ruedas
R = 0.02      # radio de rueda
MAX_SPEED = 6.28  # velocidad angular máxima de rueda


def v_robot(vl, vr):
    return ((vr + vl) / 2) * R


def omega_robot(vl, vr):
    return ((vr - vl) * R) / L


def wheel_speeds(v, omega):
    vl = (v - (omega * L) / 2) / R
    vr = (v + (omega * L) / 2) / R

    faster = max(abs(vl), abs(vr))
    if faster > MAX_SPEED:
        scale = MAX_SPEED / faster
        vl *= scale
        vr *= scale

    return vl, vr


class Odometry:

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self._prev_left = None
        self._prev_right = None

    def update(self, left_encoder_value, right_encoder_value):
        if self._prev_left is None:
            self._prev_left = left_encoder_value
            self._prev_right = right_encoder_value
            return self.x, self.y, self.theta, 0.0

        delta_left = left_encoder_value - self._prev_left
        delta_right = right_encoder_value - self._prev_right
        self._prev_left = left_encoder_value
        self._prev_right = right_encoder_value

        delta_s = R * (delta_left + delta_right) / 2.0
        delta_theta = R * (delta_right - delta_left) / L

        mid_theta = self.theta + delta_theta / 2.0
        self.x += delta_s * math.cos(mid_theta)
        self.y += delta_s * math.sin(mid_theta)
        self.theta = math.atan2(math.sin(self.theta + delta_theta), math.cos(self.theta + delta_theta))

        return self.x, self.y, self.theta, delta_s
