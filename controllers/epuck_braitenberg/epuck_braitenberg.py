"""Braitenberg obstacle avoidance for the e-puck.

Each of the eight IR proximity sensors pushes the wheels with a fixed weight,
so an obstacle on the left speeds up the left wheel and the robot turns away.
A small per-robot speed bias (from the robot name) keeps the swarm from moving
in lock-step.
"""
from controller import Robot

MAX_SPEED = 6.28
# Weights per sensor (ps0..ps7) for the left and right wheel, from the classic
# e-puck Braitenberg vehicle. ps0/ps7 face forward, ps2/ps5 sideways.
W_LEFT = [-1.3, -1.0, -0.5, 0.0, 0.0, 0.5, 1.0, 1.3]
W_RIGHT = [1.3, 1.0, 0.5, 0.0, 0.0, -0.5, -1.0, -1.3]
SENSOR_RANGE = 4096.0

robot = Robot()
dt = int(robot.getBasicTimeStep())

sensors = [robot.getDevice(f"ps{i}") for i in range(8)]
for s in sensors:
    s.enable(dt)

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
for m in (left, right):
    m.setPosition(float("inf"))
    m.setVelocity(0.0)

bias = 0.85 + 0.03 * (sum(map(ord, robot.getName())) % 5)

while robot.step(dt) != -1:
    x = [min(s.getValue(), SENSOR_RANGE) / SENSOR_RANGE for s in sensors]
    vl = 0.5 + sum(w * v for w, v in zip(W_LEFT, x)) * 4
    vr = 0.5 + sum(w * v for w, v in zip(W_RIGHT, x)) * 4
    vl = max(-1.0, min(1.0, vl)) * MAX_SPEED * bias
    vr = max(-1.0, min(1.0, vr)) * MAX_SPEED * bias
    left.setVelocity(vl)
    right.setVelocity(vr)
