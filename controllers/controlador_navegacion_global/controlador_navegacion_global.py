from controller import Robot

import kinematics
from sensors import SensorSuite
from obstacle_avoidance import ObstacleAvoidance

TIME_STEP = 64
CRUISE_SPEED = kinematics.MAX_SPEED * 0.8

robot = Robot()

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

sensors = SensorSuite(robot, TIME_STEP)
odometry = kinematics.Odometry()
avoidance = ObstacleAvoidance()

while robot.step(TIME_STEP) != -1:
    left_enc, right_enc = sensors.encoder_values()
    x, y, theta, delta_s = odometry.update(left_enc, right_enc)

    moving_forward = avoidance.state in ('IDLE', 'ESCAPE')
    sensors.read(delta_s, moving_forward)

    left_speed, right_speed, active = avoidance.step(sensors)

    if not active:
        left_speed, right_speed = CRUISE_SPEED, CRUISE_SPEED

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
