"""Controlador del proyecto."""
import sys

from controller import Robot

import kinematics
import scenarios
import planner
from occupancy_grid import OccupancyGrid
from sensors import SensorSuite
from obstacle_avoidance import ObstacleAvoidance
from path_follower import PathFollower

TIME_STEP = 64
GRID_RESOLUTION = 0.05
INFLATE = scenarios.ROBOT_RADIUS + scenarios.SAFETY_MARGIN

scenario_name = sys.argv[1] if len(sys.argv) > 1 else 'escenario_simple'
scenario = scenarios.load(scenario_name)

grid = OccupancyGrid.from_scenario(scenario, GRID_RESOLUTION, INFLATE)
waypoints = planner.astar(grid, scenario['start'], scenario['goal'])

if waypoints:
    print(f"[{scenario_name}] ruta A*: {len(waypoints)} waypoints, "
          f"longitud={planner.path_length(waypoints):.2f} m")
else:
    print(f"[{scenario_name}] A* no encontró ruta hacia la meta")

follower = PathFollower(waypoints or [], goal=scenario['goal'])

robot = Robot()

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

sensors = SensorSuite(robot, TIME_STEP)
odometry = kinematics.Odometry(*scenario['start'], scenario['start_theta'])
avoidance = ObstacleAvoidance()

print(f"[{scenario_name}] navegación iniciada")
goal_announced = False
distancia_recorrida = 0.0  # suma de |delta_s| por encoders: trayectoria ejecutada, no en línea recta

while robot.step(TIME_STEP) != -1:
    left_enc, right_enc = sensors.encoder_values()
    x, y, theta, delta_s = odometry.update(left_enc, right_enc)
    distancia_recorrida += abs(delta_s)

    moving_forward = avoidance.state in ('IDLE', 'ESCAPE')
    sensors.read(delta_s, moving_forward)

    left_speed, right_speed, active = avoidance.step(sensors)

    if not active:
        v, omega = follower.step(x, y, theta)
        left_speed, right_speed = kinematics.wheel_speeds(v, omega)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    if waypoints and follower.done and not goal_announced:
        goal_announced = True
        print(f"[{scenario_name}] meta alcanzada en t={robot.getTime():.1f}s "
              f"pose_est=({x:.2f},{y:.2f}) distancia_recorrida={distancia_recorrida:.2f}m")
