from kinematics import MAX_SPEED

THRESHOLD_HIGH = 350.0  # umbral de distancia frontal para activar el giro
TURN_DURATION = 10      # pasos de giro en el lugar
ESCAPE_DURATION = 8     # pasos de avance forzado tras el giro
STUCK_RETRIES = 1
REVERSE_DURATION = 12   # pasos de retroceso en línea recta
BACKOFF_DURATION = 6    # pasos de retroceso previos al pivote

FORWARD_SPEED = MAX_SPEED * 0.8
TURN_SPEED = MAX_SPEED * 0.3
REVERSE_SPEED = MAX_SPEED * 0.5
BACKOFF_SPEED = MAX_SPEED * 0.4


class ObstacleAvoidance:
    def __init__(self):
        self.state = 'IDLE'
        self._turn_steps = 0
        self._last_direction = None
        self._retry_count = 0

    def _turn_direction(self, sensors):
        diff = sensors.left_side - sensors.right_side
        if abs(diff) >= 50:
            return 'TURN_RIGHT' if diff > 0 else 'TURN_LEFT'
        # Desempate frontal
        if sensors.ps[0].getValue() >= sensors.ps[7].getValue():
            return 'TURN_LEFT'
        return 'TURN_RIGHT'

    def _choose_direction(self, sensors):
        if self._last_direction is not None and self._retry_count >= 1:
            return 'TURN_LEFT' if self._last_direction == 'TURN_RIGHT' else 'TURN_RIGHT'
        return self._turn_direction(sensors)

    def step(self, sensors):
        if self.state == 'IDLE':
            if sensors.d_hat >= THRESHOLD_HIGH:
                self.state = 'BACKOFF'
                self._turn_steps = 0
            else:
                return 0.0, 0.0, False

        if self.state == 'BACKOFF':
            self._turn_steps += 1
            if self._turn_steps >= BACKOFF_DURATION:
                self.state = self._choose_direction(sensors)
                self._last_direction = self.state
                self._turn_steps = 0

        elif self.state in ('TURN_LEFT', 'TURN_RIGHT'):
            self._turn_steps += 1
            if self._turn_steps >= TURN_DURATION:
                self.state = 'ESCAPE'
                self._turn_steps = 0

        elif self.state == 'ESCAPE':
            self._turn_steps += 1
            if sensors.d_hat >= THRESHOLD_HIGH:
                self._retry_count += 1
                if self._retry_count > STUCK_RETRIES:
                    self.state = 'REVERSE'
                else:
                    self.state = 'BACKOFF'
                self._turn_steps = 0
            elif self._turn_steps >= ESCAPE_DURATION:
                self.state = 'IDLE'
                self._turn_steps = 0
                self._last_direction = None
                self._retry_count = 0
                return 0.0, 0.0, False

        elif self.state == 'REVERSE':
            self._turn_steps += 1
            if self._turn_steps >= REVERSE_DURATION:
                self.state = 'IDLE'
                self._turn_steps = 0
                self._last_direction = None
                self._retry_count = 0
                return 0.0, 0.0, False

        if self.state == 'TURN_RIGHT':
            return TURN_SPEED, -TURN_SPEED, True
        if self.state == 'TURN_LEFT':
            return -TURN_SPEED, TURN_SPEED, True
        if self.state in ('REVERSE', 'BACKOFF'):
            speed = REVERSE_SPEED if self.state == 'REVERSE' else BACKOFF_SPEED
            return -speed, -speed, True
        # ESCAPE
        return FORWARD_SPEED, FORWARD_SPEED, True
