# Parámetros de filtrado (idénticos a Lab2)
ALPHA = 0.3      # coeficiente del filtro EMA
Q = 5.0          # varianza de ruido de proceso (Kalman)
R_NOISE = 200.0  # varianza de ruido de medición (Kalman)
IR_SCALE = 500.0  # factor de conversión avance [m] -> unidades IR


class SensorSuite:
    def __init__(self, robot, timestep):
        self.ps = []
        for i in range(8):
            sensor = robot.getDevice(f'ps{i}')
            sensor.enable(timestep)
            self.ps.append(sensor)

        self.left_encoder = robot.getDevice('left wheel sensor')
        self.right_encoder = robot.getDevice('right wheel sensor')
        self.left_encoder.enable(timestep)
        self.right_encoder.enable(timestep)

        self.ema_front = 0.0
        self.d_hat = 0.0
        self._P = 100.0
        self._initialized = False

        self.front_raw = 0.0
        self.left_side = 0.0
        self.right_side = 0.0

    def encoder_values(self):
        return self.left_encoder.getValue(), self.right_encoder.getValue()

    def read(self, delta_s, moving_forward):
        self.front_raw = max(
            self.ps[0].getValue(), self.ps[7].getValue(),
            self.ps[1].getValue(), self.ps[6].getValue(),
        )
        self.left_side = self.ps[6].getValue() + self.ps[5].getValue()
        self.right_side = self.ps[1].getValue() + self.ps[2].getValue()

        if not self._initialized:
            self.ema_front = self.front_raw
            self.d_hat = self.front_raw
            self._initialized = True

        # Filtro EMA
        self.ema_front = ALPHA * self.front_raw + (1.0 - ALPHA) * self.ema_front

        # Filtro de Kalman (predicción + corrección)
        if moving_forward:
            d_hat_minus = self.d_hat + IR_SCALE * max(delta_s, 0.0)
        else:
            d_hat_minus = self.d_hat

        p_minus = self._P + Q
        k = p_minus / (p_minus + R_NOISE)
        self.d_hat = d_hat_minus + k * (self.front_raw - d_hat_minus)
        self._P = (1.0 - k) * p_minus

        return self.d_hat
