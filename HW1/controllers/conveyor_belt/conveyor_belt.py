"""Drives the belt of a Conveyor PROTO.

The belt speed (m/s) is read from the robot's customData string every step, so the
world_manager supervisor can change it live. A negative value reverses the belt.
"""
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())
name = robot.getName()

motor = robot.getDevice("belt_motor")
if motor is None:
    print(f"[conveyor_belt] {name}: device 'belt_motor' NOT FOUND - the Track has no motor")
    while robot.step(timestep) != -1:
        pass
    raise SystemExit

motor.setPosition(float("inf"))
motor.setVelocity(0.0)

last = None
while robot.step(timestep) != -1:
    try:
        speed = float(robot.getCustomData() or 0.0)
    except ValueError:
        speed = 0.0
    if speed != last:
        motor.setVelocity(speed)
        print(f"[conveyor_belt] {name}: belt speed set to {speed} m/s")
        last = speed
