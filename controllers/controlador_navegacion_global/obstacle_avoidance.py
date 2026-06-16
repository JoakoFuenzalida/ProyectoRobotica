from kinematics import MAX_SPEED

THRESHOLD_HIGH = 95.0   # umbral de distancia frontal para activar el giro
TURN_DURATION = 10      # pasos de giro en el lugar
ESCAPE_DURATION = 8     # pasos de avance forzado tras el giro

FORWARD_SPEED = MAX_SPEED * 0.8
TURN_SPEED = MAX_SPEED * 0.55
INNER_SPEED = 0.0


class ObstacleAvoidance:
    def __init__(self):
        self.state = 'IDLE'
        self._turn_steps = 0

    def _turn_direction(self, sensors):
        diff = sensors.left_side - sensors.right_side
        if abs(diff) >= 50:
            return 'TURN_RIGHT' if diff > 0 else 'TURN_LEFT'
        # Desempate frontal
        if sensors.ps[0].getValue() >= sensors.ps[7].getValue():
            return 'TURN_LEFT'
        return 'TURN_RIGHT'

    def step(self, sensors):
        if self.state == 'IDLE':
            if sensors.d_hat >= THRESHOLD_HIGH:
                self.state = self._turn_direction(sensors)
                self._turn_steps = 0
            else:
                return 0.0, 0.0, False

        if self.state in ('TURN_LEFT', 'TURN_RIGHT'):
            self._turn_steps += 1
            if self._turn_steps >= TURN_DURATION:
                self.state = 'ESCAPE'
                self._turn_steps = 0

        elif self.state == 'ESCAPE':
            self._turn_steps += 1
            if sensors.d_hat >= THRESHOLD_HIGH:
                self.state = self._turn_direction(sensors)
                self._turn_steps = 0
            elif self._turn_steps >= ESCAPE_DURATION:
                self.state = 'IDLE'
                self._turn_steps = 0
                return 0.0, 0.0, False

        if self.state == 'TURN_RIGHT':
            return TURN_SPEED, INNER_SPEED, True
        if self.state == 'TURN_LEFT':
            return INNER_SPEED, TURN_SPEED, True
        # ESCAPE
        return FORWARD_SPEED, FORWARD_SPEED, True
