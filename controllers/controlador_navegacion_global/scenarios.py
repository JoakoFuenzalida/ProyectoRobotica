"""Configuración por escenario
"""

ROBOT_RADIUS = 0.037
SAFETY_MARGIN = 0.04

# start_theta: rumbo inicial del robot en el mundo
SCENARIOS = {
    'escenario_simple': {
        'start': (0.11647, 0.881111),
        'start_theta': -1.6150914722134857,
        'goal': (0.0, -1.5),
        'bounds': (-2.5, 2.5, -2.5, 2.5),
        'obstacles': [
            (0.0, 0.05, 0.3, 0.3),
            (0.95, 0.05, 0.3, 0.3),
            (-0.99, 0.05, 0.3, 0.3),
        ],
    },
    'escenario_complejo': {
        'start': (-2.2, 0.0),
        'start_theta': 0.0,
        'goal': (2.14, 1.66),
        'bounds': (-2.5, 2.5, -2.5, 2.5),
        'obstacles': [
            (-1.6, 1.325, 0.1, 1.175),    # wall1_upper (puerta en y=0, 30cm)
            (-1.6, -1.325, 0.1, 1.175),   # wall1_lower
            (-0.8, 1.825, 0.1, 0.675),    # wall2_upper (puerta en y=0.9125, ~47cm)
            (-0.8, -0.825, 0.1, 1.5),     # wall2_lower (half_y real tras el clamp de Webots a 3m)
            (0.0, 1.015, 0.1, 1.5),       # wall3_upper (cy y half_y reales tras el clamp; puerta en y=-0.7175, ~46cm)
            (0.0, -1.725, 0.1, 0.775),    # wall3_lower
            (0.8, 1.625, 0.1, 0.875),     # wall4_upper (puerta en y=0.6, 30cm)
            (0.8, -1.025, 0.1, 1.475),    # wall4_lower
            (1.6, 1.175, 0.1, 1.325),     # wall5_upper (puerta en y=-0.3, 30cm)
            (1.6, -1.475, 0.1, 1.025),    # wall5_lower
        ],
    },
}


def load(name):
    if name not in SCENARIOS:
        raise ValueError(f"Escenario desconocido: {name!r}. Opciones: {list(SCENARIOS)}")
    return SCENARIOS[name]
